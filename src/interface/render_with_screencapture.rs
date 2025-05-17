use bevy::{
    prelude::*,
    render::view::screenshot::{save_to_disk, Capturing, Screenshot},
    window::SystemCursorIcon,
    winit::cursor::CursorIcon,
};
use bevy_flycam::prelude::*;
use crate::calculator::point_filter::ransac_ground_segmentation;
use crate::data_reader::read_from_file::read_point_cloud_from_csv;
use crate::calculator::rectangle_mesh::{RectangleMesh, compare_rectangle_mesh};

pub fn run_bevy(oak_config: crate::config::OakConfig) {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(NoCameraPlayerPlugin)
        .insert_resource(oak_config.clone())
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            oak_config: Res<crate::config::OakConfig>|
            setup(commands, materials, meshes, oak_config.clone())
        )
        .add_systems(Update, (screenshot_on_q, screenshot_saving))
        .add_systems(Update, darw_gizmos)
        .run();
}

fn screenshot_on_q(
    mut commands: Commands,
    input: Res<ButtonInput<KeyCode>>,
    mut counter: Local<u32>,
) {
    if input.just_pressed(KeyCode::KeyQ) {
        let path = format!("./out/screenshot-{}.png", *counter);
        *counter += 1;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}

fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    oak_config: crate::config::OakConfig
) {
    // Load the point cloud data from a CSV file
    let points = read_point_cloud_from_csv(&oak_config.process_config.point_cloud_path, &oak_config).unwrap();
    let (ground_points, _non_ground_points) = ransac_ground_segmentation(
        &points,
        110,
        0.10,
    );

    // Load original data from a CSV file
    let origin_points = read_point_cloud_from_csv(&oak_config.process_config.origin_data_path, &oak_config).unwrap();
    let (origin_ground_points, _origin_non_ground_points) = ransac_ground_segmentation(
        &origin_points,
        110,
        0.10,
    );

    // Create a RectangleMesh from the points
    let cell_size = 0.08; // Define the size of each grid cell
    let mut rectangle_mesh = RectangleMesh::from_points(&ground_points, cell_size);
    println!("Compute finished with {} grids", rectangle_mesh.vertices.len());

    // Create a mesh from the original points
    let mut origin_rectangle_mesh = RectangleMesh::from_points(&origin_ground_points, cell_size);
    println!("Compute finished with {} grids", origin_rectangle_mesh.vertices.len());

    // Compare the two meshes
    compare_rectangle_mesh(&mut rectangle_mesh, &mut origin_rectangle_mesh, &oak_config);

    // Create a mesh from the RectangleMesh
    let mesh = rectangle_mesh.to_bevy_mesh();
    //let origin_mesh = origin_rectangle_mesh.to_bevy_mesh();

    let mesh_handle = meshes.add(mesh.clone());
    //let origin_mesh_handle = meshes.add(origin_mesh);
    let material_handle = materials.add(
        StandardMaterial {
            ..default()
        }
    );

    commands.spawn((
        Mesh3d(mesh_handle.clone()),
        MeshMaterial3d(material_handle.clone()),
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
    ));

    // commands.spawn((
    //     Mesh3d(origin_mesh_handle.clone()),
    //     MeshMaterial3d(material_handle.clone()),
    //     Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
    // ));

    // Add a camera
    // 点云旋转时由于为绕坐标原点旋转，所以此时点云包围盒位于[(), ()]
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(
            oak_config.render_config.cam_x,
            oak_config.render_config.cam_y,
            oak_config.render_config.cam_z,
        ).looking_at(Vec3::new(
            oak_config.render_config.look_at_x,
            oak_config.render_config.look_at_y,
            oak_config.render_config.look_at_z,
        ), Vec3::Y),
        FlyCam
    ));

    // Add a light source
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            intensity: 10_000_000.,
            range: 100.0,
            shadow_depth_bias: 0.2,
            ..default()
        },
        Transform::from_xyz(8.0, 16.0, 8.0),
    ));

    commands.spawn((
        Text::new("Press <Q> to save a screenshot to disk"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

fn darw_gizmos(
    mut gizmos: Gizmos,
) {
    use std::f32::consts::PI;
    let cell_size = 1.; // Define the size of each grid cell
    gizmos.grid(
        Quat::from_rotation_x(PI / 2.),
        UVec2::splat(10),
        Vec2::new(cell_size, cell_size),
        // Light gray
        LinearRgba::gray(0.35),
    );
}