use avian2d::prelude::*;
use bevy::{input::mouse::AccumulatedMouseScroll, math::VectorSpace, prelude::*, transform};

use crate::{
    camera::ToFollow,
    car::{Car, CarInput, spawn_car},
};

pub struct PlayerPlugin;
impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedPreUpdate, player_movement)
            .add_systems(Startup, spawn_player);
    }
}

#[derive(Component, PartialEq)]
pub struct Player;

fn spawn_player(mut commands: Commands) {
    commands.spawn((spawn_car(Vec2::ZERO), Player, ToFollow));
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
