using Godot;

namespace CostFunctions;

public class SocialForcesAvoidance: CostFunction
{
    private float _dt = 2;
    private float _V0 = 2.1f;
    private float _sigma = 0.3f;
    private float _U0 = 10;
    private float _R = 0.2f;
    
    private float _viewingAngleHalf = (float)(100.0 * Mathf.Pi / 180.0);
    private float _scaleOutsideView = 0.5f;
    
    public SocialForcesAvoidance(Agent agent, float weight) : base(agent, weight) {}

    public override Vector3 CalculateCostGradient(Vector3 velocity)
    {
        var position = _agent.Position;
        float rangeSquared = _agent.Range * _agent.Range;

        var agentForces = Vector3.Zero;
        
        foreach (var neighborAgent in _agent.NeighborAgents)
        {
            // GD.Print(_agent.Name+" nb: "+neighborAgent.Name+" "+(neighborAgent.Position-position).Length());
            if ((neighborAgent.Position-position).LengthSquared()<=rangeSquared)
            {
                agentForces += ComputeAgentInteractionForce(neighborAgent);
            }
        }

        var origin = _agent.Position + Vector3.Up * 0.2f;
        // DebugDraw3D.DrawArrow(origin, origin+agentForces*5, new Color(0, 1, 0), 0.2f);
        
        // GD.Print(_agent.Name+": "+agentForces+" V: "+_agent.Velocity);

        var obstacleForces = Vector3.Zero;
        foreach (var nearestPoint in _agent.NeighborObstacleNearestPoints)
        {
            if ((nearestPoint-position).LengthSquared()<=rangeSquared)
            {
                obstacleForces += ComputeObstacleInteractionForce(nearestPoint);
            }
        }
        
        // GD.Print("obstacles force: "+obstacleForces);

        return agentForces + obstacleForces;
    }

    private Vector3 ComputeAgentInteractionForce(Agent otherAgent)
    {
        Vector3 R = _agent.Position - otherAgent.Position;
        float magR = R.Length();
        // GD.Print(magR);

        Vector3 Vb = _agent.Velocity - otherAgent.Velocity;
        Vector3 V = Vb * _dt;
        float magV = V.Length();

        Vector3 RminV = R - V;
        float magRminV = RminV.Length();
        float bSquared2 = (magR + magRminV) * (magR + magRminV) - magV * magV;
        if (bSquared2<=0)
        {
            return Vector3.Zero;
        }

        float b = 0.5f * Mathf.Sqrt(bSquared2);

        Vector3 force = _V0 / _sigma * Mathf.Exp(-b / _sigma) * (magR + magRminV) / (4 * b) *
                        (R / magR + RminV / magRminV);
        
        float scale = _agent.Velocity.AngleTo(-R) < _viewingAngleHalf ? 1 : _scaleOutsideView;
        
        return scale * force;
    }

    private Vector3 ComputeObstacleInteractionForce(Vector3 nearestPoint)
    {
        Vector3 diff = _agent.Position - nearestPoint;
        float dist = diff.Length();
        if (dist<0.001f)
        {
            return Vector3.Zero;
        }
        
        var force = diff * (_U0 * Mathf.Exp(-dist / _R) / (_R * dist));
        float scale = _agent.Velocity.AngleTo(-diff) < _viewingAngleHalf ? 1 : _scaleOutsideView;
        
        return scale * force;
    }
}