use bevy::prelude::*;
use bevy::render::render_resource::{
    Extent3d,
    TextureDescriptor,
    TextureDimension,
    TextureFormat,
    TextureUsages,
};
use bevy::render::{
    camera::RenderTarget,
    RenderPlugin,
};
use bevy_flycam::prelude::*;
use bevy_image_export::{
    self,
    ImageExportPlugin,
    ImageExportSource,
    ImageExport,
    ImageExportSettings
};
use crate::calculator::point_filter::ransac_ground_segmentation;
use crate::data_reader::read_from_file::read_point_cloud_from_csv;
use crate::calculator::rectangle_mesh::RectangleMesh;

pub fn run_bevy(oak_config: crate::config::OakConfig) {
    let export_plugin = ImageExportPlugin::default();
    let export_thread = export_plugin.threads.clone();

    App::new()
        .add_plugins((
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        resolution: (
                            oak_config.render_config.width as f32,
                            oak_config.render_config.height as f32,
                        ).into(),
                        ..default()
                    }),
                    ..default()
                })
                .set(RenderPlugin {
                    synchronous_pipeline_compilation: true,
                    ..default()
                }),
            export_plugin,
        ))
        .add_plugins(NoCameraPlayerPlugin)
        .insert_resource(oak_config.clone())
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            export_sources: ResMut<Assets<ImageExportSource>>,
            images: ResMut<Assets<Image>>,
            oak_config: Res<crate::config::OakConfig>|
                setup(commands, materials, meshes, export_sources, images, oak_config.clone())
        )
        .add_systems(Update, darw_gizmos)
        .run();

    export_thread.finish();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut export_sources: ResMut<Assets<ImageExportSource>>,
    mut images: ResMut<Assets<Image>>,
    oak_config: crate::config::OakConfig
) {
    // Set up the camera and rendering configuration
    let output_texture_handle = {
        let size = Extent3d {
            width: oak_config.render_config.width,
            height: oak_config.render_config.height,
            ..default()
        };
        let mut export_texture = Image {
            texture_descriptor: TextureDescriptor {
                label: None,
                size,
                dimension: TextureDimension::D2,
                format: TextureFormat::Rgba8UnormSrgb,
                mip_level_count: 1,
                sample_count: 1,
                usage: TextureUsages::COPY_DST
                    | TextureUsages::COPY_SRC
                    | TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            },
            ..default()
        };
        export_texture.resize(size);

        images.add(export_texture)
    };

    // Load the point cloud data from a CSV file
    let points = read_point_cloud_from_csv("data/data-1-1.csv", &oak_config).unwrap();
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
    // 点云旋转时由于为绕坐标原点旋转，所以此时点云包围盒位于[(), ()]
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(
            oak_config.render_config.cam_x,
            oak_config.render_config.cam_y,
            oak_config.render_config.cam_z,
        ).looking_at(Vec3::new(0.5, -2.5, 1.7), Vec3::Y),
    ))
    .with_child((
        Camera3d::default(),
        Camera {
            // Connect the output texture to a camera as a RenderTarget.
            target: RenderTarget::Image(output_texture_handle.clone()),
            ..default()
        },
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

    // Spawn the ImageExport component to initiate the export of the output texture.
    commands.spawn((
        ImageExport(export_sources.add(output_texture_handle)),
        ImageExportSettings {
            // Frames will be saved to "./out/[#####].png".
            output_dir: "out".into(),
            // Choose "exr" for HDR renders.
            extension: "png".into(),
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