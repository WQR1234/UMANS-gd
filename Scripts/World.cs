using Godot;
using System;
using System.Collections.Generic;

public partial class World : Node
{
	public static World Instance { get; private set; }

	public static bool IsPause = false;

	public List<Agent> AllAgents;

	private PackedScene _agentScene;
	private PackedScene _wallScene;

	private NavigationRegion3D _navigationRegion;
	
	public bool StartNav { get; private set; }

	public float DeltaTime { get; private set; } = 1.0f / 60.0f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		AllAgents = new List<Agent>();
		Instance = this;

		_agentScene = GD.Load<PackedScene>("res://Scenes/agent.tscn");
		_wallScene = GD.Load<PackedScene>("res://Scenes/wall.tscn");

		_navigationRegion = GetNode<NavigationRegion3D>("/root/Main/NavigationRegion3D");
		// GD.Print(_navigationRegion.Name);
		
	}

	public override void _PhysicsProcess(double delta)
	{
		base._PhysicsProcess(delta);

		if (IsPause) return;
		DeltaTime = (float)delta;

		foreach (var agent in AllAgents)
		{
			agent.UpdateVelocityAndPosition(delta);
			
		}
	}

	public void CreatAgent(Vector3 position, Vector3 goal, float preferredSpeed, float maxSpeed, Agent.PolicyType method)
	{
		var agentInstance = _agentScene.Instantiate() as Agent;
		if (agentInstance == null)
		{
			GD.PrintErr("creat agent fail!");
			return;
		}
		agentInstance.Position = position;
		agentInstance.Goal = goal;
		agentInstance.PreferredSpeed = preferredSpeed;
		agentInstance.MaxSpeed = maxSpeed;
		agentInstance.OptMethod = method;
		agentInstance.Name = "agent" + AllAgents.Count;
		
		AddChild(agentInstance);
	}

	public void CreatWall(Vector3 position, Vector3 size, int id=0)
	{
		var wallInstance = _wallScene.Instantiate() as StaticBody3D;
		if (wallInstance == null)
		{
			return;
		}

		// wallInstance.Position = new Vector3(position.X, 0, position.Y);
		// wallInstance.Scale = new Vector3(size.X, 1, size.Y);
		// GD.Print(position, size);

		wallInstance.Position = position;
		wallInstance.Scale = size;
		
		// var wallMeshInstance = wallInstance.GetNode<MeshInstance3D>("MeshInstance3D");
		// var wallMesh = wallMeshInstance.Mesh as BoxMesh;
		// wallMesh.Size = size;
		// // wallInstance.Scale = size;
		//
		// var wallCollisionShape = wallInstance.GetNode<CollisionShape3D>("CollisionShape3D");
		// var wallShape = wallCollisionShape.Shape as BoxShape3D;
		// wallShape.Size = size;
		
		wallInstance.Name = "wall" + id;
		
		_navigationRegion.AddChild(wallInstance);
	}

	public void CreatHill()
	{
		
	}

	public void BakeNavMesh(bool startNav)
	{
		if (startNav)
		{
			_navigationRegion.BakeNavigationMesh(false);
		}

		StartNav = startNav;
	}
}
