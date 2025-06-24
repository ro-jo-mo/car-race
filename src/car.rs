use std::f32::consts::PI;

use avian2d::prelude::*;
use bevy::{
    math::{
        NormedVectorSpace, VectorSpace,
        ops::{acos, log2, tanh},
    },
    prelude::*,
};

use crate::player::Player;

const STEER_SPEED: f32 = 1.9;
const MAX_TURN_ANGLE: f32 = 22.0 * PI / 180.0;
const TURN_RADIUS_COEFFICIENT: f32 = 20.0;
const TURN_SPEED: f32 = 25000.0;
const TURN_ACCELERATION: f32 = 1000.0;
const BASE_TURN_RADIUS: f32 = 30.0;
const ACCELERATION: f32 = 700.0;
const FORWARDS_DRAG: f32 = 1.5;
const SIDE_DRAG: f32 = 8.0;

pub struct CarPlugin;
impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            FixedUpdate,
            (
                turn_wheels,
                car_acceleration,
                car_turning,
                transfer_velocity_to_wheel_direction,
            )
                .chain(),
        );
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
    let com = Vec2::new(0.0, (SIZE.y / 6.0) * 0.5);

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
        direction.0 = direction.0.clamp(-MAX_TURN_ANGLE, MAX_TURN_ANGLE);
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

fn car_acceleration(
    mut query: Query<(&CarInput, &Transform, &mut LinearVelocity), With<Car>>,
    time: Res<Time>,
) {
    for (input, transform, mut linear_velocity) in query.iter_mut() {
        let forwards = (transform.rotation * Vec3::Y).truncate();

        let acceleration = forwards * time.delta_secs() * ACCELERATION * input.input.y;

        linear_velocity.0 += acceleration;

        // Apply drag to the vehicle
        // More drag is applied to side directions
        let velocity = linear_velocity.0;
        let mut velocity = Vec3::new(velocity.x, velocity.y, 0.0);
        velocity = transform.rotation.inverse() * velocity;

        let mut drag = Vec3::ZERO;
        drag.x = velocity.x * SIDE_DRAG;
        drag.y = velocity.y * FORWARDS_DRAG;
        drag *= time.delta_secs();
        velocity -= drag;

        velocity = transform.rotation * velocity;
        linear_velocity.0 = Vec2::new(velocity.x, velocity.y);
    }
}

fn car_turning(
    mut query: Query<
        (
            &Transform,
            &LinearVelocity,
            &mut AngularVelocity,
            &WheelDirection,
        ),
        With<Car>,
    >,
    time: Res<Time>,
) {
    // Rotate using physics
    // Apply more drag based on car direction

    for (transform, linear_velocity, mut angular_velocity, wheel_direction) in query.iter_mut() {
        let forwards_magnitude = (calculate_steering_angle(*transform, wheel_direction).inverse()
            * Vec3::new(linear_velocity.0.x, linear_velocity.0.y, 0.0))
        .y;

        // Turn radius = (1 - steering angle) * forwards velocity / delta time
        // When the wheel is not turned, the forces on the front and rear wheels are equal, and we don't turn
        let turn_radius = BASE_TURN_RADIUS
            + TURN_RADIUS_COEFFICIENT * forwards_magnitude
                / wheel_direction.0.abs()
                / time.delta_secs();
        let distance_covered = forwards_magnitude * time.delta_secs();
        // NOTE Given this turning angle, attempt to push the angular velocity to it

        let mut desired_angle = (PI - 2.0 * f32::atan(turn_radius / distance_covered))
            * wheel_direction.0.signum()
            * steering_curve(forwards_magnitude.abs())
            * forwards_magnitude.signum()
            * TURN_SPEED;

        if f32::is_nan(desired_angle) {
            desired_angle = 0.0;
        }
        // Push angular velocity towards this value
        let difference = desired_angle - angular_velocity.0;
        let direction = difference.signum();

        let mut acceleration = direction * TURN_ACCELERATION * time.delta_secs();
        if acceleration.abs() > difference.abs() {
            acceleration = difference;
        }

        angular_velocity.0 += acceleration;
    }
}

fn calculate_steering_angle(transform: Transform, wheel_direction: &WheelDirection) -> Quat {
    transform.rotation * Quat::from_axis_angle(Vec3::Z, wheel_direction.0)
}

fn calculate_steering_direction(
    transform: Transform,
    wheel_direction: &WheelDirection,
    direction: Vec3,
) -> Vec2 {
    let steering_angle = calculate_steering_angle(transform, wheel_direction);
    (steering_angle * direction).truncate()
}

fn transfer_velocity_to_wheel_direction(
    mut query: Query<(&Transform, &mut LinearVelocity, &WheelDirection), With<Car>>,
    time: Res<Time>,
) {
    for (transform, mut linear_velocity, wheel_direction) in query.iter_mut() {
        let forwards = calculate_steering_direction(*transform, wheel_direction, Vec3::Y);
        let velocity_magnitude = linear_velocity.0.norm();
        if velocity_magnitude == 0.0 {
            continue;
        }
        // Transfer some velocity to the steering direction
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
fn transfer_curve(x: f32) -> f32 {
    -tanh(x * 0.015) + 1.0
}
fn steering_curve(x: f32) -> f32 {
    log2(0.3 * x + 1.0)
}
