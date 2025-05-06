use bevy::prelude::*;
use bevy_flycam::prelude::*;
use crate::calculator::point_filter::ransac_ground_segmentation;
use crate::data_reader::read_from_file::read_point_cloud_from_csv;
use crate::calculator::rectangle_mesh::RectangleMesh;


pub fn run_bevy() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(NoCameraPlayerPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, darw_gizmos)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Load the point cloud data from a CSV file
    let points = read_point_cloud_from_csv("data/data-3-2.csv").unwrap();
    let (ground_points, _non_ground_points) = ransac_ground_segmentation(
        &points,
        110,
        0.10,
    );

    // Create a RectangleMesh from the points
    let cell_size = 0.08; // Define the size of each grid cell
    let rectangle_mesh = RectangleMesh::from_points(&ground_points, cell_size);
    println!("Compute finished with {} grids", rectangle_mesh.vertices.len());

    // Create a mesh from the RectangleMesh
    let mesh = rectangle_mesh.to_bevy_mesh();

    let mesh_handle = meshes.add(mesh.clone());
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

    // Add a camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 4., 4.).looking_at(Vec3::ZERO, Vec3::Z),
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