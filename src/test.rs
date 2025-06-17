use avian2d::prelude::*;
use bevy::prelude::*;

pub struct TestPlugin;
impl Plugin for TestPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn)
            .add_systems(FixedUpdate, update);
    }
}
#[derive(Component)]
struct Test;
fn spawn(mut commands: Commands) {
    commands.spawn((
        Test,
        RigidBody::Dynamic,
        Sprite::from_color(Color::WHITE, Vec2::ONE * 30.0),
    ));
}

fn update(query: Single<(&mut LinearVelocity, &mut AngularVelocity), With<Test>>) {
    let (mut linear, mut angular) = query.into_inner();
    linear.0 = Vec2::new(0.0, 30.0);
    angular.0 = 1.0;
}
