use avian2d::prelude::*;
use bevy::prelude::*;
use pause::PausePlugin;

use crate::{car::CarPlugin, player::PlayerPlugin, test::TestPlugin};

mod car;
mod pause;
mod player;
mod test;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                mode: bevy::window::WindowMode::BorderlessFullscreen(MonitorSelection::Current),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::hsl(0.0, 0.0, 0.85)))
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(PausePlugin)
        .add_plugins(CarPlugin)
        .add_plugins(PlayerPlugin)
        .add_plugins(PhysicsDebugPlugin::default())
        //.add_plugins(TestPlugin)
        .run();
}
