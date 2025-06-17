use avian2d::prelude::*;
use bevy::{input::mouse::AccumulatedMouseScroll, math::VectorSpace, prelude::*, transform};

use crate::car::{Car, CarInput, spawn_car};

pub struct PlayerPlugin;
impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedPreUpdate, player_movement)
            .add_systems(Startup, spawn_player)
            .add_systems(Update, camera_zoom);
    }
}

#[derive(Component, PartialEq)]
pub struct Player;

fn spawn_player(mut commands: Commands) {
    commands.spawn((spawn_car(Vec2::ZERO), Player));
    commands.spawn((Camera2d::default(), Transform::from_xyz(0.0, 0.0, -10.0)));
}

fn player_movement(
    query: Single<&mut CarInput, (With<Car>, With<Player>)>,
    key_input: Res<ButtonInput<KeyCode>>,
) {
    let mut input = query.into_inner();
    input.input = get_movement_vector(key_input)
}

fn get_movement_vector(key_input: Res<ButtonInput<KeyCode>>) -> Vec2 {
    let mut out = Vec2::new(0.0, 0.0);
    if key_input.pressed(KeyCode::KeyA) {
        out.x += 1.0;
    }
    if key_input.pressed(KeyCode::KeyD) {
        out.x -= 1.0;
    }
    if key_input.pressed(KeyCode::KeyW) {
        out.y += 1.0;
    }
    if key_input.pressed(KeyCode::KeyS) {
        out.y -= 1.0;
    }
    out
}

fn camera_zoom(
    query: Single<&mut Projection, With<Camera2d>>,
    scroll: Res<AccumulatedMouseScroll>,
) {
    let mut result = query.into_inner();

    let Projection::Orthographic(ref mut projection) = *result else {
        panic!("Error: not an orthographic camera");
    };

    const SCROLL_MODIFIER: f32 = 0.15;
    let mut scale = projection.scale;

    scale /= 1.0 + SCROLL_MODIFIER * scroll.delta.y;
    scale = scale.clamp(0.1, 3.0);

    projection.scale = scale;
}
