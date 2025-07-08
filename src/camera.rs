use bevy::{input::mouse::AccumulatedMouseScroll, math::ops::powf, prelude::*};

const FOLLOW_SPEED: f32 = 0.2;
const SCROLL_MODIFIER: f32 = 0.15;

#[derive(Component)]
pub struct ToFollow;
pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_camera)
            .add_systems(Update, camera_follow)
            .add_systems(Update, camera_zoom);
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((Camera2d::default(), Transform::from_xyz(0.0, 0.0, -10.0)));
}

fn camera_follow(
    to_follow: Single<&Transform, With<ToFollow>>,
    camera: Single<&mut Transform, (With<Camera2d>, Without<ToFollow>)>,
    time: Res<Time>,
) {
    let to_follow = to_follow.into_inner();
    let mut camera = camera.into_inner();

    // Just half distance between the two points each second
    let start = camera.translation;
    let end = to_follow.translation;
    let direction = end - start;
    let direction = direction * powf(FOLLOW_SPEED, time.delta_secs());
    camera.translation += direction;
}

fn camera_zoom(
    camera: Single<&mut Projection, With<Camera2d>>,
    scroll: Res<AccumulatedMouseScroll>,
) {
    let mut result = camera.into_inner();

    let Projection::Orthographic(ref mut projection) = *result else {
        panic!("Error: not an orthographic camera");
    };

    let mut scale = projection.scale;

    scale /= 1.0 + SCROLL_MODIFIER * scroll.delta.y;
    scale = scale.clamp(0.1, 4.0);

    projection.scale = scale;
}
