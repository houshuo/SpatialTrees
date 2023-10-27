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
    }

    public void OnUpdate(ref SystemState state)
    {
        var targetQuery = SystemAPI.QueryBuilder().WithAll<PostTransformMatrix>().WithNone<Target, BVHSettings>().Build();
        var bvhQuery = SystemAPI.QueryBuilder().WithAll<PostTransformMatrix, Target>().Build();

        var targetEntities = targetQuery.ToEntityArray(state.WorldUpdateAllocator);
        var targetPostMatrices = targetQuery.ToComponentDataArray<PostTransformMatrix>(state.WorldUpdateAllocator);

        NativeArray<BoundingVolume> boundingVolumes = new NativeArray<BoundingVolume>(targetEntities.Length, Allocator.Temp);
        // init KD tree
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
        //var queryBVHTree = new QueryBVHTree
        //{
        //    Tree = tree,
        //    TargetEntities = targetEntities,
        //    TargetHandle = SystemAPI.GetComponentTypeHandle<Target>(),
        //    LocalTransformHandle = SystemAPI.GetComponentTypeHandle<LocalTransform>(true)
        //};
        //state.Dependency = queryBVHTree.ScheduleParallel(kdQuery, state.Dependency);
        //state.Dependency.Complete();
        //tree.Dispose();

        state.Dependency.Complete();
    }
}

//[BurstCompile]
//public struct QueryBVHTree : IJobChunk
//{
//    [ReadOnly] public NativeArray<Entity> TargetEntities;
//    public BVHTree Tree;

//    public ComponentTypeHandle<Target> TargetHandle;
//    [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformHandle;

//    public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
//        in v128 chunkEnabledMask)
//    {
//        var targets = chunk.GetNativeArray(ref TargetHandle);
//        var transforms = chunk.GetNativeArray(ref LocalTransformHandle);

//        NativePriorityHeap<KDTree.Neighbour> Neighbours = new NativePriorityHeap<KDTree.Neighbour>(1, Allocator.Temp);
//        for (int i = 0; i < chunk.Count; i++)
//        {
//            Neighbours.Clear();
//            Tree.GetEntriesInRange(-1, transforms[i].Position, float.MaxValue,
//                ref Neighbours);
//            var nearest = Neighbours.Peek().index;
//            targets[i] = new Target { Value = TargetEntities[nearest] };
//        }
//        Neighbours.Dispose();
//    }
//}
