use crate::constants;
use glam::{IVec2, UVec2, Vec2};
use rand::Rng;
use rapier2d::{
    na::{OPoint, Vector2},
    prelude::*,
};
use raylib::prelude::*;

pub const FRAMES_PER_SECOND: u32 = 144;

pub struct State {
    pub running: bool,
    pub time_since_last_update: f32,
    pub physics: Physics,
}

pub struct Physics {
    pub gravity: Vector2<f32>,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub physics_hooks: (),
    pub event_handler: (),

    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
}

impl State {
    pub fn new() -> Self {
        // physics init
        let gravity = vector![0.0, 9.81];
        let integration_parameters = IntegrationParameters::default();
        let mut physics_pipeline = PhysicsPipeline::new();
        let mut island_manager = IslandManager::new();
        let mut broad_phase = BroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut impulse_joint_set = ImpulseJointSet::new();
        let mut multibody_joint_set = MultibodyJointSet::new();
        let mut ccd_solver = CCDSolver::new();
        let physics_hooks = ();
        let event_handler = ();

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        // stage physics init
        //  ground is a segment at the bottom of the screen
        let ground_collider = ColliderBuilder::segment(
            Point::new(0.0, constants::DIMS.y as f32),
            Point::new(constants::DIMS.x as f32, constants::DIMS.y as f32 - 1.0),
        )
        .build();
        collider_set.insert(ground_collider);

        // left wall
        let left_wall_collider = ColliderBuilder::segment(
            Point::new(0.0, 0.0),
            Point::new(0.0, constants::DIMS.y as f32),
        )
        .build();
        collider_set.insert(left_wall_collider);

        // right wall
        let right_wall_collider = ColliderBuilder::segment(
            Point::new(constants::DIMS.x as f32, 0.0),
            Point::new(constants::DIMS.x as f32, constants::DIMS.y as f32),
        )
        .build();
        collider_set.insert(right_wall_collider);

        // top wall
        let top_wall_collider = ColliderBuilder::segment(
            Point::new(0.0, 0.0),
            Point::new(constants::DIMS.x as f32, 0.0),
        )
        .build();
        collider_set.insert(top_wall_collider);

        // let ball_collider = ColliderBuilder::ball(10.0).restitution(0.9).build();
        // let ball_rigid_body = RigidBodyBuilder::dynamic()
        //     .translation(vector![constants::DIMS.x as f32 / 2.0, 10.0])
        //     .build();
        // let ball_body_handle = rigid_body_set.insert(ball_rigid_body);
        // collider_set.insert_with_parent(ball_collider, ball_body_handle, &mut rigid_body_set);

        // spawn stuff
        let mut rng = rand::thread_rng();

        // spawn a bunch of balls
        // let num_balls = 100;
        // for _ in 0..num_balls {
        //     let ball_collider = ColliderBuilder::ball(4.0).restitution(0.9).build();
        //     let ball_rigid_body = RigidBodyBuilder::dynamic()
        //         .translation(vector![
        //             rng.gen_range(0.0..constants::DIMS.x as f32),
        //             rng.gen_range(0.0..constants::DIMS.y as f32)
        //         ])
        //         .build();
        //     let ball_body_handle = rigid_body_set.insert(ball_rigid_body);
        //     collider_set.insert_with_parent(ball_collider, ball_body_handle, &mut rigid_body_set);
        // }

        // spawn a bunch of boxes
        let num_boxes = 500;
        for _ in 0..num_boxes {
            let box_collider = ColliderBuilder::cuboid(2.0, 2.0).restitution(0.9).build();
            let box_rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![
                    rng.gen_range(0.0..constants::DIMS.x as f32),
                    rng.gen_range(0.0..constants::DIMS.y as f32)
                ])
                .build();
            let box_body_handle = rigid_body_set.insert(box_rigid_body);
            collider_set.insert_with_parent(box_collider, box_body_handle, &mut rigid_body_set);
        }

        Self {
            running: true,
            time_since_last_update: 0.0,
            physics: Physics {
                gravity,
                integration_parameters,
                physics_pipeline,
                island_manager,
                broad_phase,
                narrow_phase,
                impulse_joint_set,
                multibody_joint_set,
                ccd_solver,
                physics_hooks,
                event_handler,

                rigid_body_set,
                collider_set,
            },
        }
    }
}

pub fn process_events_and_input(rl: &mut RaylibHandle, state: &mut State) {
    if rl.is_key_pressed(raylib::consts::KeyboardKey::KEY_ESCAPE) {
        state.running = false;
    }
}

pub fn step(rl: &mut RaylibHandle, rlt: &mut RaylibThread, state: &mut State) {
    let physics = &mut state.physics;
    physics.physics_pipeline.step(
        &physics.gravity,
        &physics.integration_parameters,
        &mut physics.island_manager,
        &mut physics.broad_phase,
        &mut physics.narrow_phase,
        &mut physics.rigid_body_set,
        &mut physics.collider_set,
        &mut physics.impulse_joint_set,
        &mut physics.multibody_joint_set,
        &mut physics.ccd_solver,
        None,
        &physics.physics_hooks,
        &physics.event_handler,
    );
}

pub fn draw(state: &State, d: &mut RaylibTextureMode<RaylibDrawHandle>) {
    d.draw_text("Low Res Sketch!", 12, 12, 12, Color::WHITE);
    let mouse_pos = d.get_mouse_position();
    d.draw_circle(mouse_pos.x as i32, mouse_pos.y as i32, 6.0, Color::GREEN);

    let angle = d.get_time() as f32;

    let center = Vec2::new(d.get_screen_width() as f32, d.get_screen_height() as f32) / 2.0;
    let offset = center / 4.0;

    for i in 0..3 {
        let rot = glam::Mat2::from_angle(angle + i as f32 * 90.0);
        let rect_pos_rotated = rot * offset + center;

        let size =
            (((d.get_time() as f32 + i as f32 * 1.0) * 2.0).sin() + 1.0) / 2.0 * offset.y + 4.0;
        d.draw_rectangle(
            rect_pos_rotated.x as i32,
            rect_pos_rotated.y as i32,
            size as i32,
            size as i32,
            Color::RED,
        );
    }

    // render all colliders
    for (handle, collider) in state.physics.collider_set.iter() {
        let shape = collider.shape();
        let shape_type = shape.shape_type();
        match shape_type {
            ShapeType::Ball => {
                let ball = shape.as_ball().unwrap();
                let radius = ball.radius;
                let position = collider.position().translation.vector;
                d.draw_circle(
                    position.x as i32,
                    position.y as i32,
                    radius as f32,
                    Color::BLUE,
                );
            }
            ShapeType::Segment => {
                let segment = shape.as_segment().unwrap();
                let a = segment.a;
                let b = segment.b;
                d.draw_line(a.x as i32, a.y as i32, b.x as i32, b.y as i32, Color::BLUE);
            }
            ShapeType::Cuboid => {
                let cuboid = shape.as_cuboid().unwrap();
                let size = cuboid.half_extents * 2.0;
                let position = collider.position().translation.vector;
                d.draw_rectangle(
                    position.x as i32,
                    position.y as i32,
                    size.x as i32,
                    size.y as i32,
                    Color::BLUE,
                );
            }
            _ => {
                println!("unhandled shape type: {:?}", shape_type);
            }
        }
    }
}
