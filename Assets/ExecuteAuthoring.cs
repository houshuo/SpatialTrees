using UnityEngine;
using Unity.Entities;
using UnityEngine.Serialization;

namespace Miscellaneous.Execute
{
    public class ExecuteAuthoring : MonoBehaviour
    {
        public bool ClosestTarget;
        public bool BVH;

        class Baker : Baker<ExecuteAuthoring>
        {
            public override void Bake(ExecuteAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);
                if (authoring.ClosestTarget) AddComponent<ClosestTarget>(entity);
                if (authoring.BVH) AddComponent<BVH>(entity);
            }
        }
    }

    public struct ClosestTarget : IComponentData
    {
    }

    public struct BVH : IComponentData
    {
    }
}
