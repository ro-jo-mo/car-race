use bevy::{
    prelude::*,
    window::{CursorGrabMode, PrimaryWindow},
};

pub struct PausePlugin;

impl Plugin for PausePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_pause)
            .add_systems(Update, toggle_pause);
    }
}

#[derive(Resource)]
struct IsPaused {
    paused: bool,
}

fn setup_pause(mut commands: Commands) {
    commands.insert_resource(IsPaused { paused: false });
}

fn toggle_pause(
    keys: Res<ButtonInput<KeyCode>>,
    mut exit: EventWriter<AppExit>,
    mut is_paused: ResMut<IsPaused>,
    mut window_query: Query<&mut Window, With<PrimaryWindow>>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        is_paused.paused = !is_paused.paused;
        exit.write(AppExit::Success);
    }
    let mut window = window_query.single_mut().unwrap();
    window.cursor_options.grab_mode = CursorGrabMode::Locked;
    window.cursor_options.visible = false;
}
