use crate::constants;
use glam::{IVec2, UVec2, Vec2};
use rand::Rng;
use rapier2d::dynamics::RevoluteJoint;
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

    pub car_handle: RigidBodyHandle,
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

        // make a chain bridge from 25% of width to 75% of width, at 50% of height
        // Define the starting and ending points of the bridge
        let bridge_start = vector![
            0.25 * constants::DIMS.x as f32,
            0.5 * constants::DIMS.y as f32
        ];
        let bridge_end = vector![
            0.75 * constants::DIMS.x as f32,
            0.5 * constants::DIMS.y as f32
        ];

        let world_anchor = vector![0.0, 0.0];

        let bridge_start_collider = ColliderBuilder::ball(10.0).restitution(0.9).build();
        let bridge_start_rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![bridge_start.x, bridge_start.y])
            .build();
        let bridge_start_body_handle = rigid_body_set.insert(bridge_start_rigid_body);
        collider_set.insert_with_parent(
            bridge_start_collider,
            bridge_start_body_handle,
            &mut rigid_body_set,
        );

        let bridge_end_collider = ColliderBuilder::ball(10.0).restitution(0.9).build();
        let bridge_end_rigid_body = RigidBodyBuilder::fixed()
            .translation(vector![bridge_end.x, bridge_end.y])
            .build();
        let bridge_end_body_handle = rigid_body_set.insert(bridge_end_rigid_body);
        collider_set.insert_with_parent(
            bridge_end_collider,
            bridge_end_body_handle,
            &mut rigid_body_set,
        );

        // Decide on the number of links
        let num_links = 10;
        let link_length = (bridge_end.x - bridge_start.x) / num_links as f32;

        // Iterate over the number of links to create each one
        // Iterate over the number of links to create each one
        let mut previous_handle: Option<RigidBodyHandle> = None;
        for i in 0..num_links {
            // Calculate the position of the current link
            let x = bridge_start.x + i as f32 * link_length;
            let position = vector![x, bridge_start.y];

            // Create the rigid body and collider for the current link
            let link_collider = ColliderBuilder::cuboid(link_length * 0.5, 0.2)
                .restitution(0.9)
                .build();
            let link_rigid_body = RigidBodyBuilder::dynamic().translation(position).build();
            let link_body_handle = rigid_body_set.insert(link_rigid_body);
            collider_set.insert_with_parent(link_collider, link_body_handle, &mut rigid_body_set);

            // If this is the first or last link, create a joint to pin it in place
            if i == 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Point::from(vector![link_length * 0.5, 0.0]))
                    .local_anchor2(Point::from(world_anchor))
                    .build();
                impulse_joint_set.insert(bridge_start_body_handle, link_body_handle, joint, true);
            } else if i == num_links - 1 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Point::from(vector![-link_length * 0.5, 0.0]))
                    .local_anchor2(Point::from(world_anchor))
                    .build();
                impulse_joint_set.insert(bridge_end_body_handle, link_body_handle, joint, true);
            }

            // Create a joint between the previous link and the current one
            if let Some(prev_handle) = previous_handle {
                // let anchor_prev = vector![-link_length * 0.5, 0.0]; // Local anchor at the right end of the previous link
                // let anchor_curr = vector![link_length * 0.5, 0.0]; // Local anchor at the left end of the current link
                let anchor_prev: Point<f32> = Point::from(vector![-link_length * 0.5, 0.0]);
                let anchor_curr: Point<f32> = Point::from(vector![link_length * 0.5, 0.0]);

                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(anchor_prev)
                    .local_anchor2(anchor_curr);

                impulse_joint_set.insert(prev_handle, link_body_handle, joint, true);
            }
            previous_handle = Some(link_body_handle);
        }

        // make a car on the left side
        //// car consists of 2 rolling wheels connected by a box.
        let car_body_handle = {
            let car_collider = ColliderBuilder::cuboid(10.0, 2.0).restitution(0.9).build();
            let car_rigid_body = RigidBodyBuilder::dynamic()
                .translation(vector![10.0, 10.0])
                .build();
            let car_body_handle = rigid_body_set.insert(car_rigid_body);
            collider_set.insert_with_parent(car_collider, car_body_handle, &mut rigid_body_set);
            car_body_handle
        };

        // Left wheel position
        let left_wheel_position = vector![10.0 - 10.0 - 8.0, 10.0];

        // Right wheel position
        let right_wheel_position = vector![10.0 + 10.0 + 8.0, 10.0];

        let left_wheel_body_handle = {
            let left_wheel_collider = ColliderBuilder::ball(4.0).restitution(0.9).build();
            let left_wheel_rigid_body = RigidBodyBuilder::dynamic()
                .translation(left_wheel_position)
                .build();
            let left_wheel_body_handle = rigid_body_set.insert(left_wheel_rigid_body);
            collider_set.insert_with_parent(
                left_wheel_collider,
                left_wheel_body_handle,
                &mut rigid_body_set,
            );
            left_wheel_body_handle
        };

        let right_wheel_body_handle = {
            let right_wheel_collider = ColliderBuilder::ball(4.0).restitution(0.9).build();
            let right_wheel_rigid_body = RigidBodyBuilder::dynamic()
                .translation(right_wheel_position)
                .build();
            let right_wheel_body_handle = rigid_body_set.insert(right_wheel_rigid_body);
            collider_set.insert_with_parent(
                right_wheel_collider,
                right_wheel_body_handle,
                &mut rigid_body_set,
            );
            right_wheel_body_handle
        };

        // left wheel joint
        {
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(Point::from(vector![0.0, 0.0]))
                .local_anchor2(Point::from(vector![0.0, 0.0]))
                .build();
            impulse_joint_set.insert(car_body_handle, left_wheel_body_handle, joint, true);
        }

        // right wheel joint
        {
            let joint = RevoluteJointBuilder::new()
                .local_anchor1(Point::from(vector![10.0, 0.0]))
                .local_anchor2(Point::from(vector![0.0, 0.0]))
                .build();
            impulse_joint_set.insert(car_body_handle, right_wheel_body_handle, joint, true);
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

                car_handle: car_body_handle,
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

    // render all rigid bodies
    // render all rigid bodies
    for (handle, rigid_body) in state.physics.rigid_body_set.iter() {
        // Get the position of the rigid body.
        let position = rigid_body.position().translation.vector;

        // Here, I'll render a small circle to represent the center of mass of the body.
        let radius = 1.0; // Choose a small value for visualization purposes
        d.draw_circle(
            position.x as i32,
            position.y as i32,
            radius as f32,
            Color::RED,
        );
    }
}
