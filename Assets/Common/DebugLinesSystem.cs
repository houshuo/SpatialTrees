using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial struct DebugLinesSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        foreach (var (transform, target, entity) in SystemAPI.Query<RefRW<LocalToWorld>, RefRO<Target>>().WithEntityAccess())
        {
            if (SystemAPI.Exists(target.ValueRO.Value))
            {
                var targetTransform = SystemAPI.GetComponent<LocalToWorld>(target.ValueRO.Value);
                Debug.DrawLine(transform.ValueRO.Position, targetTransform.Position);
            }
        }
    }
}