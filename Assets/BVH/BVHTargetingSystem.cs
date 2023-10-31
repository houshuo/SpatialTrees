using System.Collections.Generic;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using DH2.Algorithm;

struct AabbOverlapLeafProcessor : BVHTree.IAabbOverlapLeafProcessor<NativeArray<Entity>>
{
    public EntityCommandBuffer.ParallelWriter ECB;
    public Entity QueryEntity;
    public int frameCount;
    public void AabbLeaf(BVHTree.OverlapAabbInput input, int leafData, ref NativeArray<Entity> allTargetEntities)
    {
        ECB.SetComponent(QueryEntity.Index, allTargetEntities[leafData], new Target(){Value = QueryEntity});
        
        // Debug.Log($"{frameCount}######################query: {QueryEntity} target: {allTargetEntities[leafData]}");
    }
}

[UpdateAfter(typeof(LocalToWorldSystem))]
public partial struct BVHTargetingSystem : ISystem
{
    BVHTree tree;
    public void OnCreate(ref SystemState state)
    {
        tree = new BVHTree();
        tree.Init();

        state.RequireForUpdate<BVHSettings>();
        state.RequireForUpdate<Miscellaneous.Execute.BVH>();
    }

    public void OnDestroy(ref SystemState state)
    {
        tree.Dispose();
    }

    public void OnUpdate(ref SystemState state)
    {
        //Clear results
        foreach (var target in SystemAPI.Query<RefRW<Target>>())
        {
            target.ValueRW.Value = new Entity();
        }
        
        //Query Entities
        var bvhQuery = SystemAPI.QueryBuilder().WithAll<PostTransformMatrix>().WithNone<Target, BVHSettings>().Build();
        //Target Entities
        var targetQuery = SystemAPI.QueryBuilder().WithAll<LocalToWorld, Target>().Build();
        var allTargetEntities = targetQuery.ToEntityArray(state.WorldUpdateAllocator);
        var targetPostMatrices = targetQuery.ToComponentDataArray<LocalToWorld>(state.WorldUpdateAllocator);

        NativeArray<BoundingVolume> boundingVolumes = new NativeArray<BoundingVolume>(targetPostMatrices.Length, Allocator.Temp);
        // init BVH
        for (int i = 0; i < targetPostMatrices.Length; i += 1)
        {
            float4x4 m = targetPostMatrices[i].Value;
            boundingVolumes[i] = new BoundingVolume
            {
                Position = new float3(m.c3.x, m.c3.y, m.c3.z),
                Extend = new float3
                (
                    math.length(new float3(m.c0.x, m.c1.x, m.c2.x)),
                    math.length(new float3(m.c0.y, m.c1.y, m.c2.y)),
                    math.length(new float3(m.c0.z, m.c1.z, m.c2.z))
                )
            };
        }
        state.Dependency = tree.ScheduleBuildTree(boundingVolumes, state.Dependency);
        boundingVolumes.Dispose();
        
        EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.TempJob);
        EntityCommandBuffer.ParallelWriter parallelWriter = ecb.AsParallelWriter();
        //Job Implement
        var queryBVHTree = new QueryBVHTree
        {
            frameCount = Time.frameCount,
            Tree = tree,
            ECB = parallelWriter,
            AllTargetEntities = allTargetEntities,
            EntityTypeHandle = SystemAPI.GetEntityTypeHandle(),
            LocalToWorldHandle = SystemAPI.GetComponentTypeHandle<LocalToWorld>(true)
        };
        state.Dependency = queryBVHTree.ScheduleParallel(bvhQuery, state.Dependency);
        state.Dependency.Complete();
        
        //Main Thread Implement
        // foreach (var (localToWorld, queryEntity) in SystemAPI.Query<RefRO<LocalToWorld>>()
        //              .WithNone<Target, BVHSettings>()
        //              .WithEntityAccess())
        // {
        //     
        //     AabbOverlapLeafProcessor processor = new AabbOverlapLeafProcessor() { ECB = parallelWriter, QueryEntity = queryEntity, frameCount = Time.frameCount};
        //     var m = localToWorld.ValueRO.Value;
        //     float3 center = new float3(m.c3.x, m.c3.y, m.c3.z);
        //     float3 extend = new float3(
        //         math.length(new float3(m.c0.x, m.c1.x, m.c2.x)),
        //         math.length(new float3(m.c0.y, m.c1.y, m.c2.y)),
        //         math.length(new float3(m.c0.z, m.c1.z, m.c2.z))
        //     );
        //     
        //     tree.AabbOverlap(
        //         new BVHTree.OverlapAabbInput { Aabb = new Aabb { Max = center + extend * 0.5f, Min = center - extend * 0.5f}, Filter = CollisionFilter.Default },
        //         ref processor, 
        //         ref allTargetEntities
        //     );
        // };
        
        ecb.Playback(state.EntityManager);
        ecb.Dispose();
    }
}

[BurstCompile]
public struct QueryBVHTree : IJobChunk
{
    [NativeDisableParallelForRestriction] public NativeArray<Entity> AllTargetEntities;
    public EntityCommandBuffer.ParallelWriter ECB;
    public BVHTree Tree;
    [ReadOnly] public EntityTypeHandle EntityTypeHandle;
    [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldHandle;
    public int frameCount;
    
    public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
        in v128 chunkEnabledMask)
    {
        var queryEntities = chunk.GetNativeArray(EntityTypeHandle);
        var localToWorlds = chunk.GetNativeArray(ref LocalToWorldHandle);
        
        for (int i = 0; i < chunk.Count; i++)
        {
            Entity queryEntity = queryEntities[i];
            AabbOverlapLeafProcessor processor = new AabbOverlapLeafProcessor() { ECB = ECB, QueryEntity = queryEntity, frameCount = frameCount};
            float4x4 m = localToWorlds[i].Value;
            float3 center = new float3(m.c3.x, m.c3.y, m.c3.z);
            float3 extend = new float3(
                math.length(new float3(m.c0.x, m.c1.x, m.c2.x)),
                math.length(new float3(m.c0.y, m.c1.y, m.c2.y)),
                math.length(new float3(m.c0.z, m.c1.z, m.c2.z))
            );
            Tree.AabbOverlap(
                new BVHTree.OverlapAabbInput { Aabb = new Aabb { Max = center + extend * 0.5f, Min = center - extend * 0.5f}, Filter = CollisionFilter.Default },
                ref processor, 
                ref AllTargetEntities
            );
        }
        
    }
}
