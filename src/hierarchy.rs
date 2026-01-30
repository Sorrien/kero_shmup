use hecs::{Entity, World};

pub struct TransformComponent {
    pub transform: glamx::Affine2,
}

pub struct ParentComponent {
    pub parent: Entity,
    pub local_transform: glamx::Affine2,
}

fn evaluate_relative_transforms(world: &mut World) {
    let mut parents = world.query::<&ParentComponent>();
    let parents = parents.view();

    let mut roots = world
        .query::<&TransformComponent>()
        .without::<&ParentComponent>();
    let roots = roots.view();

    for (parent, absolute) in world
        .query::<(&ParentComponent, &mut TransformComponent)>()
        .iter()
    {
        let mut relative = parent.local_transform;
        let mut ancestor = parent.parent;
        while let Some(next) = parents.get(ancestor) {
            relative = next.local_transform * relative;
            ancestor = next.parent;
        }
        absolute.transform = roots.get(ancestor).unwrap().transform * relative;
    }
}