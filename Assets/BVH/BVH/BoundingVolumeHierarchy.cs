using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Jobs;

namespace DH2.Algorithm
{
    public struct BoundingVolume
    {
        public float3 Position;
        public float3 Extend;
        //public CollisionFilter Filter;
    }

    public struct BVHTree : IDisposable, ICloneable
    {
        private NativeArray<BoundingVolumeHierarchy.Node> Nodes; // The nodes of the bounding volume
        private NativeArray<CollisionFilter> NodeFilters;        // The collision filter for each node (a union of all its children)
        private BoundingVolumeHierarchy BoundingVolumeHierarchy;
        
        private int m_BodyCount;
        public int BodyCount
        {
            get { return m_BodyCount; }
            set { m_BodyCount = value; NodeCount = value + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches; }
        }

        private int m_NodeCount;
        private int NodeCount
        {
            get => m_NodeCount;
            set
            {
                m_NodeCount = value;
                if (value > Nodes.Length)
                {
                    if(Nodes.IsCreated) Nodes.Dispose();
                    Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(value, Allocator.Persistent, NativeArrayOptions.UninitializedMemory)
                    {
                        // Always initialize first 2 nodes as empty, to gracefully return from queries on an empty tree
                        [0] = BoundingVolumeHierarchy.Node.Empty,
                        [1] = BoundingVolumeHierarchy.Node.Empty
                    };

                    if (NodeFilters.IsCreated) NodeFilters.Dispose();
                    NodeFilters = new NativeArray<CollisionFilter>(value, Allocator.Persistent, NativeArrayOptions.UninitializedMemory)
                    {
                        // All queries should descend past these special root nodes
                        [0] = CollisionFilter.Default,
                        [1] = CollisionFilter.Default
                    };

                    BoundingVolumeHierarchy = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                }
            }
        }

        private NativeArray<int> m_BranchCount;
        public int BranchCount { get => m_BranchCount[0]; set => m_BranchCount[0] = value; }

        public void Init()
        {
            m_BodyCount = 0;
            m_BranchCount = new NativeArray<int>(1, Allocator.Persistent, NativeArrayOptions.ClearMemory);
        }

        public object Clone()
        {
            var clone = new BVHTree();
            clone.Nodes = new NativeArray<BoundingVolumeHierarchy.Node>(Nodes.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            clone.Nodes.CopyFrom(Nodes);
            clone.NodeFilters = new NativeArray<CollisionFilter>(NodeFilters.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            clone.NodeFilters.CopyFrom(NodeFilters);
            clone.m_BranchCount = new NativeArray<int>(1, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            clone.BranchCount = BranchCount;
            clone.m_BodyCount = BodyCount;
            return clone;
        }

        public void Dispose()
        {
            if (Nodes.IsCreated)
            {
                Nodes.Dispose();
            }

            if (NodeFilters.IsCreated)
            {
                NodeFilters.Dispose();
            }

            if (m_BranchCount.IsCreated)
            {
                m_BranchCount.Dispose();
            }
        }

        public JobHandle ScheduleBuildTree(NativeArray<BoundingVolume> treeNodes, JobHandle deps = new JobHandle())
        {
            BodyCount = treeNodes.Length;
            NativeArray<BoundingVolumeHierarchy.PointAndIndex> pointAndIndex = new NativeArray<BoundingVolumeHierarchy.PointAndIndex>(BodyCount, Allocator.TempJob);
            NativeArray<Aabb> aabbs = new NativeArray<Aabb>(BodyCount, Allocator.TempJob);
            NativeArray<CollisionFilter> filters = new NativeArray<CollisionFilter>(BodyCount, Allocator.TempJob);
            for (int i = 0; i < BodyCount; i++)
            {
                float3 center = treeNodes[i].Position;
                float3 extend = treeNodes[i].Extend;
                pointAndIndex[i] = new BoundingVolumeHierarchy.PointAndIndex() { Index = i, Position = center };
                aabbs[i] = new Aabb() { Max = center + extend / 2, Min = center - extend / 2 };
                //NodeFilters[i] = treeNodes[i].Filter;
                filters[i] = CollisionFilter.Default;
            }
            
            return BoundingVolumeHierarchy.ScheduleBuildJobs(pointAndIndex, aabbs, filters, 8, deps, NodeCount, m_BranchCount);
        }

        #region AABB overlap query
        public struct OverlapAabbInput
        {
            public Aabb Aabb;
            public CollisionFilter Filter;
        }

        public interface IAabbOverlapLeafProcessor<T>
        {
            // Called when the query overlaps a leaf node of the bounding volume hierarchy
            void AabbLeaf(OverlapAabbInput input, int leafData, ref T collector);
        }

        public unsafe void AabbOverlap<TProcessor, TCollector>(OverlapAabbInput input, ref TProcessor processor, ref TCollector collector)
            where TProcessor : struct, IAabbOverlapLeafProcessor<TCollector>
            // where TCollector : struct
        {
            int* binaryStack = stackalloc int[BoundingVolumeHierarchy.Constants.BinaryStackSize];
            int* stack = binaryStack;
            *stack++ = 1;

            FourTransposedAabbs aabbT;
            (&aabbT)->SetAllAabbs(input.Aabb);
            do
            {
                int nodeIndex = *(--stack);
                BoundingVolumeHierarchy.Node* node = (BoundingVolumeHierarchy.Node*)Nodes.GetUnsafeReadOnlyPtr() + nodeIndex;
                bool4 overlap = aabbT.Overlap1Vs4(ref node->Bounds);
                int4 compressedValues;
                int compressedCount = math.compress((int*)(&compressedValues), 0, node->Data, overlap);

                if (node->IsLeaf)
                {
                    for (int i = 0; i < compressedCount; i++)
                    {
                        processor.AabbLeaf(input, compressedValues[i], ref collector);
                    }
                }
                else
                {
                    *((int4*)stack) = compressedValues;
                    stack += compressedCount;
                }
            } while (stack > binaryStack);
        }

        #endregion

        //#region Self overlap query

        //public interface ITreeOverlapCollector
        //{
        //    void AddPairs(int l, int4 r, int countR);
        //    void AddPairs(int4 l, int4 r, int count);
        //    void FlushIfNeeded();
        //}

        //public unsafe void BvhOverlap<T>(ref T pairWriter, BoundingVolumeHierarchy other, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        //{
        //    TreeOverlap(ref pairWriter, m_Nodes, other.m_Nodes, m_NodeFilters, other.m_NodeFilters, rootA, rootB);
        //}

        //public unsafe void SelfBvhOverlap<T>(ref T pairWriter, int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        //{
        //    TreeOverlap(ref pairWriter, m_Nodes, m_Nodes, m_NodeFilters,  m_NodeFilters, rootA, rootB);
        //}

        //public static unsafe void TreeOverlap<T>(
        //    ref T pairWriter,
        //    Node* treeA, Node* treeB,
        //    CollisionFilter* collisionFilterA = null, CollisionFilter* collisionFilterB = null,
        //    int rootA = 1, int rootB = 1) where T : struct, ITreeOverlapCollector
        //{
        //    int* binaryStackA = stackalloc int[Constants.BinaryStackSize];
        //    int* binaryStackB = stackalloc int[Constants.BinaryStackSize];
        //    int* stackA = binaryStackA;
        //    int* stackB = binaryStackB;

        //    int4* compressedDataBuffer = stackalloc int4[4];

        //    if (treeA == treeB && rootA == rootB)
        //    {
        //        int* unaryStack = stackalloc int[Constants.UnaryStackSize];
        //        int* stack = unaryStack;
        //        *stack++ = rootA;

        //        do
        //        {
        //            int nodeIndex = *(--stack);
        //            if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndex], collisionFilterB[nodeIndex]))
        //            {
        //                ProcessAA(ref treeA[nodeIndex], compressedDataBuffer, ref stack, ref stackA, ref stackB, ref pairWriter);
        //            }
        //            pairWriter.FlushIfNeeded();

        //            while (stackA > binaryStackA)
        //            {
        //                int nodeIndexA = *(--stackA);
        //                int nodeIndexB = *(--stackB);

        //                if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
        //                {
        //                    ProcessAB(&treeA[nodeIndexA], &treeA[nodeIndexB], treeA, treeA, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
        //                }
        //                pairWriter.FlushIfNeeded();
        //            }
        //        } while (stack > unaryStack);
        //    }
        //    else
        //    {
        //        *stackA++ = rootA;
        //        *stackB++ = rootB;

        //        do
        //        {
        //            int nodeIndexA = *(--stackA);
        //            int nodeIndexB = *(--stackB);
        //            if (collisionFilterA == null || CollisionFilter.IsCollisionEnabled(collisionFilterA[nodeIndexA], collisionFilterB[nodeIndexB]))
        //            {
        //                ProcessAB(&treeA[nodeIndexA], &treeB[nodeIndexB], treeA, treeB, compressedDataBuffer, ref stackA, ref stackB, ref pairWriter);
        //            }

        //            pairWriter.FlushIfNeeded();
        //        } while (stackA > binaryStackA);
        //    }
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //private static unsafe void ProcessAA<T>(ref Node node, int4* compressedData, ref int* stack, ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        //{
        //    int4 nodeData = node.Data;
        //    FourTransposedAabbs nodeBounds = node.Bounds;

        //    FourTransposedAabbs* aabbT = stackalloc FourTransposedAabbs[3];
        //    aabbT[0] = nodeBounds.GetAabbT(0);
        //    aabbT[1] = nodeBounds.GetAabbT(1);
        //    aabbT[2] = nodeBounds.GetAabbT(2);

        //    bool4* masks = stackalloc bool4[3];
        //    masks[0] = new bool4(false, true, true, true);
        //    masks[1] = new bool4(false, false, true, true);
        //    masks[2] = new bool4(false, false, false, true);

        //    int3 compressedCounts = int3.zero;
        //    compressedCounts[0] = math.compress((int*)(compressedData)    , 0, nodeData, aabbT[0].Overlap1Vs4(ref nodeBounds) & masks[0]);
        //    compressedCounts[1] = math.compress((int*)(compressedData + 1), 0, nodeData, aabbT[1].Overlap1Vs4(ref nodeBounds) & masks[1]);
        //    compressedCounts[2] = math.compress((int*)(compressedData + 2), 0, nodeData, aabbT[2].Overlap1Vs4(ref nodeBounds) & masks[2]);

        //    if (node.IsLeaf)
        //    {
        //        for (int i = 0; i < 3; i++)
        //        {
        //            pairWriter.AddPairs(nodeData[i], compressedData[i], compressedCounts[i]);
        //        }
        //    }
        //    else
        //    {
        //        int4 internalNodes;
        //        int numInternals = math.compress((int*)&internalNodes, 0, nodeData, node.AreInternalsValid);
        //        *((int4*)stack) = internalNodes;
        //        stack += numInternals;

        //        for (int i = 0; i < 3; i++)
        //        {
        //            *((int4*)stackA) = new int4(nodeData[i]);
        //            *((int4*)stackB) = compressedData[i];
        //            stackA += compressedCounts[i];
        //            stackB += compressedCounts[i];
        //        }
        //    }
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //private static unsafe void ProcessAB<T>(
        //    Node* nodeA, Node* nodeB,
        //    Node* treeA, Node* treeB,
        //    int4* compressedData,
        //    ref int* stackA, ref int* stackB, ref T pairWriter) where T : struct, ITreeOverlapCollector
        //{
        //    if (nodeA->IsInternal && nodeB->IsLeaf)
        //    {
        //        Node* tmp = nodeA;
        //        nodeA = nodeB;
        //        nodeB = tmp;

        //        treeB = treeA;
        //    }

        //    bool4* overlapMask = stackalloc bool4[4];
        //    FourTransposedAabbs aabbTA = nodeA->Bounds.GetAabbT(0);
        //    overlapMask[0] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
        //    aabbTA = nodeA->Bounds.GetAabbT(1);
        //    overlapMask[1] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
        //    aabbTA = nodeA->Bounds.GetAabbT(2);
        //    overlapMask[2] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);
        //    aabbTA = nodeA->Bounds.GetAabbT(3);
        //    overlapMask[3] = aabbTA.Overlap1Vs4(ref nodeB->Bounds);

        //    int4 compressedCount = int4.zero;

        //    compressedCount[0] = math.compress((int*)&compressedData[0], 0, nodeB->Data, overlapMask[0]);
        //    compressedCount[1] = math.compress((int*)&compressedData[1], 0, nodeB->Data, overlapMask[1]);
        //    compressedCount[2] = math.compress((int*)&compressedData[2], 0, nodeB->Data, overlapMask[2]);
        //    compressedCount[3] = math.compress((int*)&compressedData[3], 0, nodeB->Data, overlapMask[3]);

        //    if (nodeA->IsLeaf && nodeB->IsLeaf)
        //    {
        //        for (int i = 0; i < 4; i++)
        //        {
        //            pairWriter.AddPairs(nodeA->Data[i], compressedData[i], compressedCount[i]);
        //        }
        //    }
        //    else if (nodeA->IsInternal && nodeB->IsInternal)
        //    {
        //        for (int i = 0; i < 4; i++)
        //        {
        //            *((int4*)stackA) = new int4(nodeA->Data[i]);
        //            *((int4*)stackB) = compressedData[i];
        //            stackA += compressedCount[i];
        //            stackB += compressedCount[i];
        //        }
        //    }
        //    else
        //    {
        //        for (int i = 0; i < 4; i++)
        //        {
        //            if (compressedCount[i] > 0)
        //            {
        //                *((int4*)stackA) = compressedData[i];
        //                int* internalStack = stackA + compressedCount[i];
        //                FourTransposedAabbs aabbT = nodeA->Bounds.GetAabbT(i);
        //                int4 leafA = new int4(nodeA->Data[i]);

        //                do
        //                {
        //                    Node* internalNode = treeB + *(--internalStack);
        //                    int4 internalCompressedData;
        //                    int internalCount = math.compress((int*)&internalCompressedData, 0, internalNode->Data, aabbT.Overlap1Vs4(ref internalNode->Bounds));
        //                    if (internalCount > 0)
        //                    {
        //                        if (internalNode->IsLeaf)
        //                        {
        //                            pairWriter.AddPairs(leafA, internalCompressedData, internalCount);
        //                            pairWriter.FlushIfNeeded();
        //                        }
        //                        else
        //                        {
        //                            *((int4*)internalStack) = internalCompressedData;
        //                            internalStack += internalCount;
        //                        }
        //                    }
        //                }
        //                while (internalStack != stackA);
        //            }
        //        }
        //    }
        //}

        //#endregion



        //#region Ray cast query

        //public interface IRaycastLeafProcessor
        //{
        //    // Called when the query hits a leaf node of the bounding volume hierarchy
        //    bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>;
        //}

        //public unsafe bool Raycast<TProcessor, TCollector>(RaycastInput input, ref TProcessor leafProcessor, ref TCollector collector)
        //    where TProcessor : struct, IRaycastLeafProcessor
        //    where TCollector : struct, ICollector<RaycastHit>
        //{
        //    bool hadHit = false;
        //    int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
        //    *top++ = 1;
        //    do
        //    {
        //        Node* node = m_Nodes + *(--top);
        //        bool4 hitMask = node->Bounds.Raycast(input.Ray, collector.MaxFraction, out float4 hitFractions);
        //        int4 hitData;
        //        int hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

        //        if (node->IsLeaf)
        //        {
        //            for (int i = 0; i < hitCount; i++)
        //            {
        //                hadHit |= leafProcessor.RayLeaf(input, hitData[i], ref collector);
        //                if (collector.EarlyOutOnFirstHit && hadHit)
        //                {
        //                    return true;
        //                }
        //            }
        //        }
        //        else
        //        {
        //            *((int4*)top) = hitData;
        //            top += hitCount;
        //        }
        //    } while (top > stack);

        //    return hadHit;
        //}

        //#endregion

        //#region Collider cast query

        //public interface IColliderCastLeafProcessor
        //{
        //    // Called when the query hits a leaf node of the bounding volume hierarchy
        //    bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector) where T : struct, ICollector<ColliderCastHit>;
        //}

        //public unsafe bool ColliderCast<TProcessor, TCollector>(ColliderCastInput input, ref TProcessor leafProcessor, ref TCollector collector)
        //    where TProcessor : struct, IColliderCastLeafProcessor
        //    where TCollector : struct, ICollector<ColliderCastHit>
        //{
        //    float3 aabbExtents;
        //    Ray aabbRay;
        //    {
        //        Aabb aabb = input.Collider->CalculateAabb(new RigidTransform(input.Orientation, input.Start));
        //        aabbExtents = aabb.Extents;
        //        aabbRay = input.Ray;
        //        aabbRay.Origin = aabb.Min;
        //    }

        //    bool hadHit = false;

        //    int* stack = stackalloc int[Constants.UnaryStackSize], top = stack;
        //    *top++ = 1;
        //    do
        //    {
        //        Node* node = m_Nodes + *(--top);
        //        FourTransposedAabbs bounds = node->Bounds;
        //        bounds.Lx -= aabbExtents.x;
        //        bounds.Ly -= aabbExtents.y;
        //        bounds.Lz -= aabbExtents.z;
        //        bool4 hitMask = bounds.Raycast(aabbRay, collector.MaxFraction, out float4 hitFractions);
        //        int4 hitData;
        //        int hitCount = math.compress((int*)(&hitData), 0, node->Data, hitMask);

        //        if (node->IsLeaf)
        //        {
        //            for (int i = 0; i < hitCount; i++)
        //            {
        //                hadHit |= leafProcessor.ColliderCastLeaf(input, hitData[i], ref collector);
        //                if (collector.EarlyOutOnFirstHit && hadHit)
        //                {
        //                    return true;
        //                }
        //            }
        //        }
        //        else
        //        {
        //            *((int4*)top) = hitData;
        //            top += hitCount;
        //        }
        //    } while (top > stack);

        //    return hadHit;
        //}

        //#endregion

        //#region Point distance query

        //public interface IPointDistanceLeafProcessor
        //{
        //    // Called when the query hits a leaf node of the bounding volume hierarchy
        //    bool DistanceLeaf<T>(PointDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        //}

        //public unsafe bool Distance<TProcessor, TCollector>(PointDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
        //    where TProcessor : struct, IPointDistanceLeafProcessor
        //    where TCollector : struct, ICollector<DistanceHit>
        //{
        //    UnityEngine.Assertions.Assert.IsTrue(collector.MaxFraction <= input.MaxDistance);

        //    bool hadHit = false;

        //    int* binaryStack = stackalloc int[Constants.BinaryStackSize];
        //    int* stack = binaryStack;
        //    *stack++ = 1;

        //    var pointT = new Math.FourTransposedPoints(input.Position);
        //    float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

        //    do
        //    {
        //        int nodeIndex = *(--stack);
        //        Node* node = m_Nodes + nodeIndex;
        //        float4 distanceToNodesSquared = node->Bounds.DistanceFromPointSquared(ref pointT);
        //        bool4 overlap = (node->Bounds.Lx <= node->Bounds.Hx) & (distanceToNodesSquared <= maxDistanceSquared);
        //        int4 hitData;
        //        int hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

        //        if (node->IsLeaf)
        //        {
        //            for (int i = 0; i < hitCount; i++)
        //            {
        //                hadHit |= leafProcessor.DistanceLeaf(input, hitData[i], ref collector);

        //                if (collector.EarlyOutOnFirstHit && hadHit)
        //                {
        //                    return true;
        //                }
        //            }

        //            maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
        //        }
        //        else
        //        {
        //            *((int4*)stack) = hitData;
        //            stack += hitCount;
        //        }
        //    } while (stack > binaryStack);

        //    return hadHit;
        //}

        //#endregion

        //#region Collider distance query

        //public interface IColliderDistanceLeafProcessor
        //{
        //    // Called when the query hits a leaf node of the bounding volume hierarchy
        //    bool DistanceLeaf<T>(ColliderDistanceInput input, int leafData, ref T collector) where T : struct, ICollector<DistanceHit>;
        //}

        //public unsafe bool Distance<TProcessor, TCollector>(ColliderDistanceInput input, ref TProcessor leafProcessor, ref TCollector collector)
        //    where TProcessor : struct, IColliderDistanceLeafProcessor
        //    where TCollector : struct, ICollector<DistanceHit>
        //{
        //    UnityEngine.Assertions.Assert.IsTrue(collector.MaxFraction <= input.MaxDistance);

        //    bool hadHit = false;

        //    int* binaryStack = stackalloc int[Constants.BinaryStackSize];
        //    int* stack = binaryStack;
        //    *stack++ = 1;

        //    Aabb aabb = input.Collider->CalculateAabb(input.Transform);
        //    FourTransposedAabbs aabbT;
        //    (&aabbT)->SetAllAabbs(aabb);
        //    float4 maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);

        //    do
        //    {
        //        int nodeIndex = *(--stack);
        //        Node* node = m_Nodes + nodeIndex;
        //        float4 distanceToNodesSquared = node->Bounds.DistanceFromAabbSquared(ref aabbT);
        //        bool4 overlap = (node->Bounds.Lx <= node->Bounds.Hx) & (distanceToNodesSquared <= maxDistanceSquared);
        //        int4 hitData;
        //        int hitCount = math.compress((int*)(&hitData), 0, node->Data, overlap);

        //        if (node->IsLeaf)
        //        {
        //            for (int i = 0; i < hitCount; i++)
        //            {
        //                hadHit |= leafProcessor.DistanceLeaf(input, hitData[i], ref collector);
        //                if (collector.EarlyOutOnFirstHit && hadHit)
        //                {
        //                    return true;
        //                }

        //                maxDistanceSquared = new float4(collector.MaxFraction * collector.MaxFraction);
        //            }
        //        }
        //        else
        //        {
        //            *((int4*)stack) = hitData;
        //            stack += hitCount;
        //        }
        //    } while (stack > binaryStack);

        //    return hadHit;
        //}

        //#endregion
    }
}
