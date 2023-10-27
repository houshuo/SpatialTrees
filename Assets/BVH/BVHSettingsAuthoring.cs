using UnityEngine;
using Unity.Entities;

public class BVHSettingsAuthoring : MonoBehaviour
{
    public int unitCount;
    public GameObject unitPrefab;

    public int targetCount;
    public GameObject targetPrefab;

    class Baker : Baker<BVHSettingsAuthoring>
    {
        public override void Bake(BVHSettingsAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, new BVHSettings
            {
                UnitCount = authoring.unitCount,
                UnitPrefab = GetEntity(authoring.unitPrefab, TransformUsageFlags.Dynamic),
                TargetCount = authoring.targetCount,
                TargetPrefab = GetEntity(authoring.targetPrefab, TransformUsageFlags.Dynamic),
            });
        }
    }
}

public struct BVHSettings : IComponentData
{
    public int UnitCount;
    public Entity UnitPrefab;

    public int TargetCount;
    public Entity TargetPrefab;
}