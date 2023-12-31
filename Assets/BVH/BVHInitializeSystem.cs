using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

public partial struct BVHInitializationSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<BVHSettings>();
        state.RequireForUpdate<Miscellaneous.Execute.BVH>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Enabled = false;

        var settings = SystemAPI.GetSingleton<BVHSettings>();
        var random = Random.CreateFromIndex(1234);

        Spawn(ref state, settings.UnitPrefab, settings.UnitCount, ref random);
        Spawn(ref state, settings.TargetPrefab, settings.TargetCount, ref random);
    }

    void Spawn(ref SystemState state, Entity prefab, int count, ref Random random)
    {
        var units = state.EntityManager.Instantiate(prefab, count, Allocator.Temp);

        for (int i = 0; i < units.Length; i += 1)
        {
            var position = new float3();
            position.xz = random.NextFloat2() * 200 - 100;
            state.EntityManager.SetComponentData(units[i],
                new LocalTransform { Position = position, Scale = 1 });
            
            state.EntityManager.SetComponentData(units[i],
                new Movement { Value = random.NextFloat2Direction() });

            state.EntityManager.AddComponent<PostTransformMatrix>(units[i]);
            state.EntityManager.SetComponentData(units[i],
                new PostTransformMatrix { Value = float4x4.Scale(random.NextFloat() * 5, 1, random.NextFloat() * 5) });
        }
    }
}

