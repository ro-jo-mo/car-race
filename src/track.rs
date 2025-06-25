use avian2d::{math::PI, prelude::*};
use bevy::{math::ops::atan2, prelude::*};

const POINTS: usize = 50;
const RANGE: f32 = 1000.0;
const THRESHOLD: f32 = 0.002;
const MIN_DISTANCE: f32 = 100.0;
const RATIO_OFFSET: f32 = 500.0;
const COLLIDER_STEPS: usize = 15;
pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, generate_track)
            .add_systems(Update, draw_track);
    }
}

#[derive(Component)]
struct Curve(CubicCurve<Vec3>);

fn generate_track(mut commands: Commands) {
    // Maybe just generate a group of random points
    // Then attempt to create a minimum area lasso around them
    // Generate points with some minimum distance between them, so the track cant overlap
    // First find the centre of the cluster
    // Start with the point furthest from the centre, point - centre = basis
    // Order points by their angle from the basis vector
    // Cycle through. Find the next point based on ratio of the angle from the basis to their distance from centre

    let mut points = generate_points();
    let centre = calculate_centre(&points);
    let basis_index = get_basis_index(&points, centre);
    let basis_vector = points[basis_index];
    sort_points_by_angle(&mut points, basis_index);

    let mut track_points = Vec::<Vec3>::new();

    // Convert to iterative method, try to find n points for the track
    // Progressively increase the threshold
    find_next_point(
        &points[..],
        basis_vector,
        Quat::from_rotation_z(PI / 2.0) * (basis_vector - centre),
        &mut track_points,
    );

    track_points.push(basis_vector);

    for i in 0..POINTS {
        let point = points[i];
        let colour = Color::hsv(0.0, 0.0, i as f32 / POINTS as f32);
        if i == 0 {
            commands.spawn((
                Transform::from_xyz(point.x, point.y, point.z),
                Sprite::from_color(Color::hsv(0.0, 1.0, 1.0), Vec2::ONE * 10.0),
            ));
        } else {
            commands.spawn((
                Transform::from_xyz(point.x, point.y, point.z),
                Sprite::from_color(colour, Vec2::ONE * 10.0),
            ));
        }
    }

    for point in track_points.iter() {
        commands.spawn((
            Transform::from_xyz(point.x, point.y, 1.0),
            Sprite::from_color(Color::hsv(0.0, 1.0, 1.0), Vec2::ONE * 10.0),
        ));
    }

    commands.spawn((
        Transform::from_xyz(centre.x, centre.y, centre.z),
        Sprite::from_color(Color::hsv(90.0, 1.0, 1.0), Vec2::ONE * 10.0),
    ));

    let curve = create_curve(track_points);
    let track_collider = create_track_collider(&curve);
    commands.spawn((curve, track_collider, RigidBody::Static));
}

fn generate_points() -> Vec<Vec3> {
    let mut points = Vec::<Vec3>::with_capacity(POINTS);
    for _ in 0..POINTS {
        // Take random points from a uniform distribution
        let point = Vec3::new(rand::random(), rand::random(), 0.0) * RANGE - (RANGE / 2.0);
        points.push(point);
    }
    points
}

fn calculate_centre(points: &Vec<Vec3>) -> Vec3 {
    points.iter().sum::<Vec3>() / POINTS as f32
}

fn get_basis_index(points: &Vec<Vec3>, centre: Vec3) -> usize {
    let mut max_dist = -1.0;
    let mut max_index = 0;
    for i in 0..POINTS {
        let point = points[i];
        let distance = centre.distance_squared(point);
        if distance > max_dist {
            max_dist = distance;
            max_index = i;
        }
    }
    max_index
}

fn sort_points_by_angle(points: &mut Vec<Vec3>, basis: usize) {
    let basis = points[basis];
    points.sort_by(|a, b| signed_angle(&basis, &a).total_cmp(&signed_angle(&basis, &b)));
}

fn signed_angle(a: &Vec3, b: &Vec3) -> f32 {
    let mut angle = atan2(a.x, a.y) - atan2(b.x, b.y);
    if angle < 0.0 {
        angle += 2.0 * PI;
    }
    angle
}

fn find_next_point(
    points: &[Vec3],
    current_point: Vec3,
    current_direction: Vec3,
    total: &mut Vec<Vec3>,
) {
    // Just take the first point to match our requirements ?
    // Ratio of angle to distance

    for i in 0..points.len() {
        let point = points[i];
        let distance = current_point.distance(point);

        if distance < MIN_DISTANCE {
            continue;
        }

        let direction = point - current_point;
        let angle = current_direction.angle_between(direction);
        let ratio = angle / (distance + RATIO_OFFSET);

        if ratio < THRESHOLD {
            total.push(point);
            if i == points.len() - 1 {
                break;
            }
            find_next_point(&points[i + 1..], point, direction, total);
            break;
        }
    }
}

fn draw_track(query: Query<&Curve>, mut gizmos: Gizmos) {
    for Curve(curve) in query.iter() {
        let resolution = 100 * curve.segments().len();
        gizmos.linestrip(curve.iter_positions(resolution), Color::srgb(1.0, 1.0, 1.0));
    }
}

fn create_curve(points: Vec<Vec3>) -> Curve {
    let spline = CubicCardinalSpline::new_catmull_rom(points);
    let curve = spline.to_curve_cyclic().unwrap();
    Curve(curve)
}

fn create_track_collider(curve: &Curve) -> Collider {
    let curve = &curve.0;
    let points = curve
        .iter_positions(COLLIDER_STEPS * curve.segments().len())
        .map(|v| v.truncate())
        .collect();
    Collider::polyline(points, Option::None)
}
