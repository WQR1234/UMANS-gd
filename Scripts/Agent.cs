using Godot;
using System;
using System.Collections.Generic;
using CostFunctions;

public partial class Agent : CharacterBody3D
{
    private Vector3 _acceleration;
    private bool _isShowLabels = false;

    [Export]
    public float MaxSpeed { get; set; } = 1.6f;

    [Export] public Vector3 Goal { get; set; } 
    /// <summary>Returns the agent's preferred walking speed.</summary>
    [Export]
    public float PreferredSpeed { get; set; } = 1.3f;


    public float MaxAcceleration { get; set; } = 5;

    public float Range { get; set; } = 5;
    
    
    /// <summary>Returns the agent's last computed preferred velocity.</summary>
    public Vector3 PreferredVelocity { get; private set; }    
    
    /// <summary>Sets this Policy's relaxation time to the given value.</summary>
    /// <summary> Returns the relaxation time of this Policy.</summary>
    public float RelaxationTime { get; private set; } = 0.5f;   

    /// <summary>
    /// 球形射线节点。Spherical rays used to detect neighbors per frame.
    /// </summary>
    private ShapeCast3D _shapeCast;

    /// <summary>
    /// 动画节点
    /// </summary>
    private AnimationPlayer _animation;

    /// <summary>
    /// 模型节点
    /// </summary>
    private Node3D _modelNode;

    /// <summary>
    /// UI节点
    /// </summary>
    private Sprite3D _spriteNode;
    private Label _posLabel;
    private Label _velLabel;

    private NavigationAgent3D _navigationAgent;
    private Vector3? _localGoal;  // 局部（临时）目标点

    private List<Agent> _neighborAgents;
    public IReadOnlyList<Agent> NeighborAgents => _neighborAgents.AsReadOnly();
    private List<Vector3> _neighborObstacleNearestPoints;
    public IReadOnlyList<Vector3> NeighborObstacleNearestPoints => _neighborObstacleNearestPoints.AsReadOnly();

    private List<CostFunction> _costFunctions;
    public IReadOnlyList<CostFunction> CostFunctions => _costFunctions.AsReadOnly();

    private Timer _trackTimer;
    private List<Vector3> _trackPoints;

    private List<Vector3> _localGoals;

    private List<Vector3> _velocityHistory;
    private bool _hasOutputCSV = false;

    public enum PolicyType
    {
        Gradient, Sampling,
    }
    /// <summary>
    /// 成本函数的求解方式
    /// </summary>
    public PolicyType OptMethod { get; set; } = PolicyType.Gradient;

    public override void _Ready()
    {
        _shapeCast = GetNode<ShapeCast3D>("ShapeCast3D");
        _modelNode = GetNode<Node3D>("Model");
        ((SphereShape3D)_shapeCast.Shape).Radius = 5;
        _shapeCast.CollisionMask = OptMethod == PolicyType.Gradient ? 3U : 1U;

        _animation = GetNode<AnimationPlayer>("Model/RootNode/AnimationPlayer");

        _spriteNode = GetNode<Sprite3D>("Sprite3D");
        _posLabel = GetNode<Label>("Sprite3D/SubViewport/VBoxContainer/PositionLabel");
        _velLabel = GetNode<Label>("Sprite3D/SubViewport/VBoxContainer/VelocityLabel");

        _navigationAgent = GetNode<NavigationAgent3D>("NavigationAgent3D");
        _navigationAgent.TargetPosition = Goal;   // 设置目标点

        _neighborAgents = new List<Agent>();
        _neighborObstacleNearestPoints = new List<Vector3>();
        _costFunctions = new List<CostFunction>();

        _trackTimer = GetNode<Timer>("TrackTimer");
        _trackPoints = new();

        _localGoals = new();
        _velocityHistory = new();
        
        // TODO: 添加该agent的cost functions
        // _costFunctions.Add(new GoalReachingForce(this, 1));
        // _costFunctions.Add(new SocialForcesAvoidance(this, 1));
        
        
        World.Instance.AllAgents.Add(this);
        
        _trackTimer.Start();


        // GD.Print("agent: "+this.Name+" ready");
    }

    /// <summary>
    /// 根据json数据添加cost function, cost function的名字必须为CostFunctions命名空间下已有的类。
    /// </summary>
    /// <param name="costFunctionData">json数据中的每1条cost function数据</param>
    public void AddCostFunction(Godot.Collections.Dictionary costFunctionData)
    {
        string costFunctionName = costFunctionData["name"].AsString();
        costFunctionName = "CostFunctions." + costFunctionName;
        
        Type costFunctionType = Type.GetType(costFunctionName);
        if (costFunctionType==null)
        {
            GD.Print($"There is no cost function {costFunctionName}.");
            return;
        }

        float weight = costFunctionData["weight"].AsSingle();
        object[] costFunctionArgs = { this, weight };

        var costFunctionInstance = Activator.CreateInstance(costFunctionType, costFunctionArgs) as CostFunction;
        
        _costFunctions.Add(costFunctionInstance);
    }

    public void AddCostFunction(UserInterface.CostFunctionData costFunctionData)
    {
        // if (costFunctionData.SamplingPara!=null)
        // {
        //     var samplingPara = costFunctionData.SamplingPara.Value;
        //     GD.Print(samplingPara.@base);
        //     GD.Print(samplingPara.radius);
        //     GD.Print(samplingPara.speedSamples);
        //     GD.Print(samplingPara.randomSamples);
        // }
        string costFunctionName = costFunctionData.Name;
        costFunctionName = "CostFunctions." + costFunctionName;
        
        Type costFunctionType = Type.GetType(costFunctionName);
        if (costFunctionType==null)
        {
            GD.Print($"There is no cost function {costFunctionName}.");
            return;
        }
        
        object[] costFunctionArgs = { this, costFunctionData.Weight };
        var costFunctionInstance = Activator.CreateInstance(costFunctionType, costFunctionArgs) as CostFunction;
        if (costFunctionInstance!=null && costFunctionData.SamplingPara.HasValue)
        {
            costFunctionInstance.samplingParams = costFunctionData.SamplingPara.Value;
        }

        _costFunctions.Add(costFunctionInstance);
    }

    public override void _PhysicsProcess(double delta)
    {
        // if (Name=="Agent2")
        // {
        //     return;
        // }

        // GetInput();
        
        // GD.Print(this.Name+" process");
        
        // ShowTrackAndTarget();
        
        // EmitRays(out _, out _, true);

        if (World.IsPause) 
        {
            _animation.Stop();
            return;
        }
        
        _velocityHistory.Add(Velocity);

        if (_isShowLabels)
        {
            UpdateAgentLabels();
        }

        ComputeNeighbors();
        
        if (World.Instance.StartNav) 
            ComputePreferredVelocityWithNav();
        else 
            ComputePreferredVelocity();
        
        ComputeAcceleration(delta);
        // UpdateVelocityAndPosition(delta);  // 在World中执行
        
        PlayAnimation();

        if (!_hasOutputCSV && HasReachedGoal())
        {
            _hasOutputCSV = true;
            // WriteVelocityToCSV();
        }

        // GD.Print(Name+" move to "+Goal);
    }
    
    private void GetInput()
    {
        Vector2 inputDir = Input.GetVector("left", "right", "up", "down");
        Velocity = new Vector3(inputDir.X, 0, inputDir.Y) * MaxSpeed;
    }

    private void ComputeNeighbors()
    {
        _neighborAgents.Clear();
        _neighborObstacleNearestPoints.Clear();
        
        // GD.Print(_shapeCast.GetCollisionCount());
        if (_shapeCast.IsColliding())
        {
            
            for (int i = 0; i < _shapeCast.GetCollisionCount(); i++)
            {
                if (_shapeCast.GetCollider(i) is Agent otherAgent)
                {
                    _neighborAgents.Add(otherAgent);
                    // GD.Print(Name+": "+otherAgent.Name);
                }
                
                else if (_shapeCast.GetCollider(i) is StaticBody3D obstacle)
                {
                    // GD.Print(_shapeCast.GetCollisionNormal(i));
                    float normalAngle = Vector3.Up.AngleTo(_shapeCast.GetCollisionNormal(i)) * 180f / Mathf.Pi;
                    // GD.Print(normalAngle);
                    if (normalAngle>45)  // 若碰撞点处法线与垂直线夹角大于45°，则视为不可经过的障碍物。
                    {
                        _neighborObstacleNearestPoints.Add(_shapeCast.GetCollisionPoint(i));
                    }
                    
                    
                    // _neighborObstacles.Add(obstacle);
                    // GD.Print(obstacle.Name);
                    // GD.Print(_shapeCast.GetCollisionNormal(i));
                }
            }
        }
        
        
        
    }

    /// <summary>
    /// 判断是否已到达目标点。若当前位置与目标点距离小于0.5，则视为已到达目标。
    /// </summary>
    /// <returns>bool值，是否到达目标点</returns>
    private bool HasReachedGoal()
    {
        return (Goal - Position).LengthSquared() < 0.25f;
    }

    private bool HasReachedGoal(Vector3 localGoal)
    {
        return (localGoal - Position).LengthSquared() < 0.25f;
    }
    
    /// <summary>
    /// 计算理想速度 （大小与方向，无导航）
    /// </summary>
    private void ComputePreferredVelocity()
    {
        if (HasReachedGoal())
        {
            PreferredVelocity = Vector3.Zero;
        }
        else
        {
            PreferredVelocity = (Goal - Position).Normalized() * PreferredSpeed;
        }
    }

    /// <summary>
    /// 根据导航计算理想速度（大小与方向）
    /// </summary>
    private void ComputePreferredVelocityWithNav()
    {
        if (_navigationAgent.IsNavigationFinished())
        {
            PreferredVelocity = Vector3.Zero;
        }
        else
        {
            // GD.Print(_localGoal);
            if (!_localGoal.HasValue)
            {
                _localGoal = _navigationAgent.GetNextPathPosition();
                GD.Print("LOCAL POINT: "+_localGoal);
                _localGoals.Add(_localGoal.Value);
                
                return;
            }
            
            if (HasReachedGoal(_localGoal.Value))
            {
                _localGoal = _navigationAgent.GetNextPathPosition();
                GD.Print("NEW LOCAL POINT: "+_localGoal);
                _localGoals.Add(_localGoal.Value);

            }
            // else
            // {
            //     GD.Print("not arrive loacl");
            // }
            PreferredVelocity = (_localGoal.Value - Position).Normalized() * PreferredSpeed;
        }
        
    }

    /// <summary>
    /// 根据cost函数，计算加速度。
    /// </summary>
    /// <param name="delta">每帧时间间隔</param>
    private void ComputeAcceleration(double delta)
    {
        _acceleration = Vector3.Zero;
        
        if (OptMethod == PolicyType.Gradient)
        {
            foreach (var costFunction in _costFunctions)
            {        
                _acceleration += costFunction.CalculateCostGradient(Velocity);        
            }
        }
        else
        {
            Vector3 bestVelocity = _costFunctions[0].ApproximateGlobalMinimumBySampling(delta);
            
            _acceleration = (bestVelocity - Velocity) / (float)delta;
        }

        if (_acceleration.LengthSquared()>25)
        {
            _acceleration = _acceleration.Normalized() * 5;
        }
        // GD.Print("a: "+acceleration);
        
        
    }

    /// <summary>
    /// 根据加速度，更新速度与位置。
    /// </summary>
    /// <param name="delta">每帧时间间隔</param>
    public void UpdateVelocityAndPosition(double delta)
    {
        Velocity += _acceleration * (float)delta;
        
        if (Velocity.LengthSquared()>MaxSpeed*MaxSpeed)
        {
            Velocity = Velocity.Normalized() * MaxSpeed;
        }

        // MoveAndSlide();
        
        var collision = MoveAndCollide(Velocity * (float)delta);
        if (collision!=null)
        {
            Velocity -= 5f*(float)delta*Velocity.Project(collision.GetNormal());
        }

        // Velocity = new Vector3(Velocity.X, 0, Velocity.Z);
    }

    /// <summary>
    /// 播放动画
    /// </summary>
    private void PlayAnimation()
    {
        // GD.Print("v: "+Velocity);

        if (Velocity.LengthSquared()>=0.025f &&
            (_trackPoints.Count == 0 || Position.DistanceSquaredTo(_trackPoints[^1]) >= 0.001f))
        {
            Vector3 target = new Vector3(Velocity.X, 0, Velocity.Z);
            _modelNode.Basis = Basis.LookingAt(target, useModelFront: true);
            _animation.Play("BasicMotions@Walk01/BasicMotions_Walk01 - Forwards");
        }
        else
        {
            _animation.Play("BasicMotions@Idle01/BasicMotions_Idle01");
        }
    }

    private void OnInputEvent(Node camera, InputEvent @event , Vector3 event_position, Vector3 normal, int shape_idx)
    {
        if (@event is InputEventMouseButton mouseButton)
        {
            if (mouseButton.ButtonIndex == MouseButton.Left)
            {
                if (!mouseButton.IsPressed())
                {
                    // GD.Print(camera.Name);
                    // GD.Print(this.Name);
                    _isShowLabels = !_isShowLabels;
                    ShowOrHideLabels(_isShowLabels);
                }
            }
            
        }
    }

    /// <summary>
    /// 显示或隐藏标签
    /// </summary>
    /// <param name="isShow">是否显示</param>
    private void ShowOrHideLabels(bool isShow)
    {
        if (isShow)
        {
            _spriteNode.Show();
            UpdateAgentLabels();
        }
            
        else
            _spriteNode.Hide();
    }

    /// <summary>
    /// 更新该Agent的标签的文本信息
    /// </summary>
    private void UpdateAgentLabels()
    {
        _posLabel.Text = "位置： "+Position.ToString("0.00");
        _velLabel.Text = "速度： "+Velocity.ToString("0.00");
    }
    
    private void OnTrackTimerTimeOut()
    {
        if (Velocity.LengthSquared()<=0.025f) return;
        if (_trackPoints.Count>0 && _trackPoints[^1].DistanceSquaredTo(Position)<0.025f) return;

        _trackPoints.Add(Position);
        
    }

    /// <summary>
    /// 显示轨迹与目标点
    /// </summary>
    private void ShowTrackAndTarget()
    {
        DebugDraw3D.DrawPoints(_trackPoints.ToArray(), DebugDraw3D.PointType.TypeSphere, 0.02f, new Color(1, 1, 0));  // yellow
        DebugDraw3D.DrawSphere(Goal, 0.05f, new Color(0, 1, 0));   // green
        
        DebugDraw3D.DrawPoints(_localGoals.ToArray(), DebugDraw3D.PointType.TypeSquare, 0.25f, new Color(1, 0, 0)); // red
    }

    public void Reset()
    {
        Velocity = Vector3.Zero;
        _localGoal = null;
        _trackPoints.Clear();
        _navigationAgent.TargetPosition = Goal; // 重置目标点，以重新生成导航路径
    }

    public bool EmitRays(out Vector3 intersection, out Vector3 normal, bool showRay=false)
    {
        Vector3 origin = Position + Vector3.Up * 0.2f;
        var spaceState = this.GetWorld3D().DirectSpaceState;
        var query = PhysicsRayQueryParameters3D.Create(origin, 
            origin+Velocity.Normalized()*Range, 
            2);
        var result = spaceState.IntersectRay(query);
        if (result.Count>0)
        {
            intersection = result["position"].AsVector3();
            normal = result["normal"].AsVector3();

            if (showRay)
            {
                DebugDraw3D.DrawArrow(origin, intersection, new Color(1, 1, 0), 0.05f);
                DebugDraw3D.DrawArrowRay(intersection, normal, 2, new Color(1, 1, 0), 0.05f);
            }

            return true;
        }

        intersection = Vector3.Zero;
        normal = Vector3.Zero;
        return false;

    }

    private void WriteVelocityToCSV()
    {
        using var csvFile = Godot.FileAccess.Open($"res://output/{Name}.csv", FileAccess.ModeFlags.Write);
        
        csvFile.StoreLine("X,Y,Z");

        foreach (var vel in _velocityHistory)
        {
            csvFile.StoreLine($"{vel.X},{vel.Y},{vel.Z}");
        }
        
        GD.Print("Data written to CSV successfully.");
    }
}
