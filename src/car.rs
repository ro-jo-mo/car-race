use std::f32::consts::PI;

use avian2d::prelude::*;
use bevy::{
    math::{
        NormedVectorSpace, VectorSpace,
        ops::{acos, tanh},
    },
    prelude::*,
};

use crate::player::Player;

const STEER_SPEED: f32 = 1.9;
const MAX_TURN_ANGLE: f32 = 22.0 * PI / 180.0;
const TURN_SPEED: f32 = 15.0;
const ACCELERATION: f32 = 500.0;
const DRAG: f32 = 7.0;
const ANGULAR_DRAG: f32 = 1.0;
// Ratio of forward drag to side drag
const DRAG_RATIO: f32 = 0.1;

pub struct CarPlugin;
impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, (turn_wheels, car_physics).chain());
    }
}

#[derive(Component)]
pub struct Car;

#[derive(Component)]
pub struct CarInput {
    pub input: Vec2,
}

#[derive(Component)]
struct WheelDirection(f32);

#[derive(Component)]
enum Wheel {
    Rear,
    Front,
}

pub fn spawn_car(position: Vec2) -> impl Bundle {
    const SIZE: Vec2 = Vec2::new(35.0, 60.0);
    const WHEEL_SIZE: Vec2 = Vec2::new(8.0, 16.0);
    let offset = Vec2::new(-4.0 + WHEEL_SIZE.x / 2.0, 5.0 + WHEEL_SIZE.y / 2.0);
    let wheel_pos = Vec2::new(SIZE.x / 2.0, SIZE.y / 2.0) - offset;
    let com = Vec2::new(0.0, (SIZE.y / 2.0) * 0.5);

    (
        Car,
        CarInput { input: Vec2::ZERO },
        WheelDirection(0.0),
        Collider::rectangle(SIZE.x, SIZE.y),
        Sprite::from_color(Color::hsl(rand::random::<f32>() * 360.0, 0.5, 0.8), SIZE),
        RigidBody::Dynamic,
        GravityScale(0.0),
        CenterOfMass::new(com.x, com.y),
        Transform::from_xyz(position.x, position.y, 0.0),
        children![
            (
                Wheel::Front,
                Transform::from_xyz(wheel_pos.x, wheel_pos.y, -1.0),
                Sprite::from_color(Color::hsl(0.0, 0.0, 0.2), WHEEL_SIZE)
            ),
            (
                Wheel::Front,
                Transform::from_xyz(-wheel_pos.x, wheel_pos.y, -1.0),
                Sprite::from_color(Color::hsl(0.0, 0.0, 0.2), WHEEL_SIZE)
            ),
            (
                Wheel::Rear,
                Transform::from_xyz(wheel_pos.x, -wheel_pos.y, -1.0),
                Sprite::from_color(Color::hsl(0.0, 0.0, 0.2), WHEEL_SIZE)
            ),
            (
                Wheel::Rear,
                Transform::from_xyz(-wheel_pos.x, -wheel_pos.y, -1.0),
                Sprite::from_color(Color::hsl(0.0, 0.0, 0.2), WHEEL_SIZE)
            )
        ],
    )
}

fn turn_wheels(
    mut cars: Query<(&CarInput, &mut WheelDirection, &Children, Option<&Player>), With<Car>>,
    mut wheels: Query<(&mut Transform, &Wheel)>,
    time: Res<Time>,
) {
    for (input, mut direction, children, is_player) in cars.iter_mut() {
        direction.0 += input.input.x * STEER_SPEED * time.delta_secs();
        if is_player != Option::None {
            if input.input.x == 0.0 {
                // Auto centre
                let sign = direction.0.signum();
                let delta = STEER_SPEED * time.delta_secs();
                direction.0 -= sign * delta;
                if direction.0.abs() <= delta {
                    direction.0 = 0.0;
                }
            }
        }
        direction.0 = direction.0.clamp(-MAX_TURN_ANGLE, MAX_TURN_ANGLE);
        for child in children {
            if let Ok((mut transform, wheel)) = wheels.get_mut(*child) {
                match wheel {
                    Wheel::Rear => {}
                    Wheel::Front => {
                        let mut rotation = transform.rotation.to_euler(EulerRot::YXZ);
                        rotation.2 = direction.0;
                        transform.rotation =
                            Quat::from_euler(EulerRot::YXZ, rotation.0, rotation.1, rotation.2);
                    }
                }
            }
        }
    }
}

fn car_physics(
    mut query: Query<
        (
            &CarInput,
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            &WheelDirection,
        ),
        With<Car>,
    >,
    time: Res<Time>,
) {
    // Rotate using physics
    // Apply more drag based on car direction

    for (input, mut transform, mut linear_velocity, mut angular_velocity, wheel_direction) in
        query.iter_mut()
    {
        // let the speed of rotation be bounded by +- max_angle
        // can't use a hard limit, i.e. in the case of collisions, the vehicle will face external forces that may go past these limits
        // Steering auto returns to centre

        // could do simple grip simulation
        // transfer some velocity to the tyre direction
        // lower the ratio of transfered velocity as the angle between desired direction, and actual velocity increases
        // high speeds should also lower this ratio
        let acceleration = Vec2::Y * time.delta_secs() * ACCELERATION * input.input.y;
        let acceleration = transform.rotation * Vec3::new(acceleration.x, acceleration.y, 0.0);
        let acceleration = acceleration.truncate();
        linear_velocity.0 += acceleration;

        // Apply drag
        // More drag to side directions
        let velocity = linear_velocity.0;
        let mut velocity = Vec3::new(velocity.x, velocity.y, 0.0);
        // Rotate velocity so the x and y axis correspond to the forward and right vectors relative to the vehicle rotation
        velocity = transform.rotation.inverse() * velocity;
        let mut drag = Vec3::ZERO;
        drag.x = velocity.x;
        drag.y = velocity.y * DRAG_RATIO;
        drag *= time.delta_secs() * DRAG;
        // Rotate back to world space
        velocity -= drag;

        // save velocity.y
        let forwards_magnitude = velocity.y;
        let side_magnitude = velocity.x.abs();

        velocity = transform.rotation * velocity;
        linear_velocity.0 = Vec2::new(velocity.x, velocity.y);

        // Turning:
        // 1. Get the amount of velocity in vehicle direction
        // 2. magnitude * steering_curve(velocity.magnitude) * TURN_SPEED = car rotation
        // Steering curve goes from 1.0 to 0.0
        angular_velocity.0 +=
            steering_curve(forwards_magnitude) * wheel_direction.0 * time.delta_secs();

        // Angular drag
        // Always have some opposition to turning
        // Very high when at a stand still
        // Past some speed threshold, the drag is fairly constant
        // -tanh ?
        angular_velocity.0 -= angular_velocity.0.signum()
            * (steering_curve(side_magnitude) + 1.0)
            * time.delta_secs();

        // Transfer some velocity to the steering direction
        let steering_angle = transform.rotation * Quat::from_axis_angle(Vec3::Z, wheel_direction.0);

        let forwards = (steering_angle * Vec3::Y).truncate();
        // notes:
        // replace turning systemwith angular acceleration and drag
        // this allows 180s etc
        // lower falloff in steering curve
        // transfer portion of velocity to steer direction based on
        // might want to clamp dot if doing a 180 results in weird behaviour
        // dot(velocity,steer dir) * (-tanh(velocity.norm) + 1)

        let velocity_magnitude = linear_velocity.0.norm();
        if velocity_magnitude == 0.0 {
            continue;
        }

        let conversion_ratio = forwards.dot(linear_velocity.0 / velocity_magnitude)
            * transfer_curve(velocity_magnitude)
            * time.delta_secs();

        let transferred = velocity_magnitude * conversion_ratio;
        let untransferred = 1.0 - conversion_ratio;

        let transferred_velocity = forwards * transferred;
        let untransferred_velocity = linear_velocity.0 * untransferred;
        linear_velocity.0 = transferred_velocity + untransferred_velocity;
    }
}
// https://www.desmos.com/calculator/fbjwusvtxg
fn steering_curve(x: f32) -> f32 {
    tanh(x * 0.001) * TURN_SPEED
}

fn transfer_curve(x: f32) -> f32 {
    -tanh(x * 0.008) + 1.0
}
