/* -*- Mode: c; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil; -*- */

#![allow(unnecessary_parens)]

use std::cmp;
use std::num;
use std::rand::{task_rng, Rng};
use std::io::{BufferedWriter, Write, File};


static NAO_SAMPLES: uint = 8;
static NSUBSAMPLES: uint = 2;

mod vector {
    use std::num;

    pub struct Vector3 {
        pub x: f32,
        pub y: f32,
        pub z: f32
    }

    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x: x, y: y, z: z }
    }

    #[inline(always)]
    pub fn new_normal(x: f32, y: f32, z: f32) -> Vector3 {
        let mut v = Vector3 { x: x, y: y, z: z };
        v.normalized();
        return v;
    }

    // operator +
    #[inline(always)]
    impl Add<Vector3, Vector3> for Vector3 {
        fn add(&self, other: &Vector3) -> Vector3 {
            Vector3 { x: self.x + other.x,
                     y: self.y + other.y,
                     z: self.z + other.z }
        }
    }

    // operator -
    #[inline(always)]
    impl Sub<Vector3, Vector3> for Vector3 {
        fn sub(&self, other: &Vector3) -> Vector3 {
            Vector3 { x: self.x - other.x,
                     y: self.y - other.y,
                     z: self.z - other.z }
        }
    }

    // operator *
    #[inline(always)]
    impl Mul<Vector3, Vector3> for Vector3 {
        fn mul(&self, other: &Vector3) -> Vector3 {
            Vector3 { x: self.x * other.x,
                     y: self.y * other.y,
                     z: self.z * other.z }
        }
    }

    #[inline(always)]
    pub fn dot(v0: &Vector3, v1: &Vector3) -> f32 {
        v0.x * v1.x + v0.y * v1.y + v0.z * v1.z
    }

    #[inline(always)]
    pub fn cross(v0: &Vector3, v1: &Vector3) -> Vector3 {
        Vector3 { x: v0.y * v1.z - v0.z * v1.y,
                 y: v0.z * v1.x - v0.x * v1.z,
                 z: v0.x * v1.y - v0.y * v1.x }
    }

    #[inline(always)]
    pub fn scale(v: &Vector3, s: f32) -> Vector3 {
        Vector3 { x: v.x * s, y: v.y * s, z: v.z * s }
    }

    impl Vector3 {
        #[inline(always)]
        pub fn normalized(&mut self) {
            let length = dot(self, self).sqrt();
            if (length < -1.0e-9) || (length > 1.0e-9) {
                self.x /= length;
                self.y /= length;
                self.z /= length;
            }
        }
    }
}

struct Ray {
    origin: vector::Vector3,
    direction: vector::Vector3
}

struct IntersectInfo {
    distance: f32,
    position: vector::Vector3,
    normal: vector::Vector3
}

enum Object {
    Sphere(vector::Vector3, f32),
    Plane(vector::Vector3, vector::Vector3)
}

#[allow(non_snake_case)]
impl Object {
    pub fn intersect(&self, ray: &Ray, isect: &mut IntersectInfo) -> bool {
        match *self {
            Sphere(position, radius) => {
                let rs = ray.origin - position;
                let B = vector::dot(&rs, &ray.direction);
                let C = vector::dot(&rs, &rs) - radius * radius;
                let D = B * B - C;
                if D > 0.0 {
                    let t = -B - D.sqrt();
                    if (t > 0.0) && (t < isect.distance) {
                        isect.distance = t;
                        isect.position = ray.origin + vector::scale(&ray.direction, t);
                        isect.normal = isect.position - position;
                        isect.normal.normalized();
                        return true;
                    }
                }
                return false;
            },
            Plane(position, normal) => {
                let d = -vector::dot(&position, &normal);
                let v = vector::dot(&ray.direction, &normal);
                if v.abs() < 1.0e-9f32 { return false; }
                let t = -(vector::dot(&ray.origin, &normal) + d) / v;
                if (t > 0.0) && (t < isect.distance) {
                    isect.distance = t;
                    isect.position = ray.origin + vector::scale(&ray.direction, t);
                    isect.normal = normal;
                    return true;
                }
                return false;
            }
        }
    }
}

// ---

#[inline(always)]
fn ortho_basis(n: vector::Vector3) -> [vector::Vector3, ..3] {
    // 'if' is not statement. it's expression.
    let basis1 =
        if (n.x < 0.6) && (n.x > -0.6) {
            vector::new(1.0, 0.0, 0.0)
        } else if ((n.y < 0.6) && (n.y > -0.6)) {
            vector::new(0.0, 1.0, 0.0)
        } else if ((n.z < 0.6) && (n.z > -0.6)) {
            vector::new(0.0, 0.0, 1.0)
        } else {
            vector::new(1.0, 0.0, 0.0)
        };
    let mut basis0 = vector::cross(&basis1, &n);
    basis0.normalized();

    let mut basis1 = vector::cross(&n, &basis0);
    basis1.normalized();

    return [basis0, basis1, n];
}

fn ambient_occlusion(isect: &IntersectInfo,
                     objects: &[Object]) -> f32 {
    let eps = 0.0001f32;
    let ntheta = NAO_SAMPLES;
    let nphi = NAO_SAMPLES;
    let mut ray_origin = isect.position + vector::scale(&isect.normal, eps);
    let basis = ortho_basis(isect.normal);
    let mut occlusion: f32 = 0.0;
    let mut occ_isect = IntersectInfo {
        distance: 1.0e+9,
        position: vector::new(0.0, 0.0, 0.0),
        normal: vector::new(0.0, 1.0, 0.0)
    };
    let tau: f32 = 2.0f32 * num::Float::pi();
    let mut rng = task_rng();

    for _ in range(0u, ntheta) {
        for _ in range(0u, nphi) {
            let theta = rng.gen::<f32>().sqrt();
            let phi = tau * rng.gen::<f32>();

            let x = phi.cos() * theta;
            let y = phi.sin() * theta;
            let z = (1.0f32 - theta * theta).sqrt();

            // local -> global
            let direction = vector::Vector3 {
                x: x * basis[0].x + y * basis[1].x + z * basis[2].x,
                y: x * basis[0].y + y * basis[1].y + z * basis[2].y,
                z: x * basis[0].z + y * basis[1].z + z * basis[2].z
            };
            let ray = Ray { origin: ray_origin,
                            direction: direction };
            occ_isect.distance = 1.0e+9;
            for o in objects.iter() {
                if o.intersect(&ray, &mut occ_isect) {
                    occlusion += 1.0;
                    break;
                }
            }
        }
    }
    let theta = ntheta as f32;
    let phi = nphi as f32;
    return (theta * phi - occlusion) / (theta * phi);
}


struct Pixel {
    r:u8, g:u8, b:u8
}

impl Pixel {
    /*
    #[inline(always)]
    pub fn new(r:u8, g:u8, b:u8) -> Pixel {
        Pixel { r:r, g:g, b:b }
    }
    */

    #[inline(always)]
    pub fn new_with_clamp(value: f32, mag: f32) -> Pixel {
        let v = (value * mag) as uint;
        let i = cmp::min(255u, v) as u8;
        return Pixel { r:i, g:i, b:i };
    }
}

fn render_line(width: uint, height: uint, _y: uint,
               nsubsamples: uint, objects: &[Object]) -> Vec<Pixel> {
    let mut line: Vec<Pixel> = Vec::with_capacity(width);
    let sample: f32 = nsubsamples as f32;
    let w: f32 = width as f32;
    let h: f32 = height as f32;
    let y: f32 = _y as f32;
    for _x in range(0u, width) {
        let mut occlusion = 0.0f32;
        for _u in range(0u, nsubsamples) {
            for _v in range(0u, nsubsamples) {
                let x: f32 = _x as f32;
                let u: f32 = _u as f32;
                let v: f32 = _v as f32;
                let px: f32 = (x + (u / sample) - (w / 2.0f32)) / (w / 2.0f32);
                let py: f32 = -(y + (v / sample) - (h / 2.0f32)) / (h / 2.0f32);
                let ray = Ray { origin: vector::new(0.0, 0.0, 0.0),
                                direction: vector::new_normal(px, py, -1.0) };

                let mut isect = IntersectInfo {
                    distance: 1.0e+17,
                    position: vector::new(0.0, 0.0, 0.0),
                    normal: vector::new(0.0, 1.0, 0.0)
                };
                let mut hit = false;
                for o in objects.iter() {
                    let h = o.intersect(&ray, &mut isect);
                    hit = (hit || h);
                }
                if hit {
                    occlusion += ambient_occlusion(&mut isect, objects);
                }
            }
        }
        if occlusion > 0.0001 {
            let c = occlusion / ((nsubsamples * nsubsamples) as f32);
            line.push(Pixel::new_with_clamp(c, 255.0));
        } else {
            line.push(Pixel { r:0u8, g:0u8, b:0u8 });
        }
    }
    return line;
}

fn render_singletask(width: uint, height: uint, nsubsamples: uint, objects: &[Object]) -> Vec<Pixel> {
    let mut lines: Vec<Pixel> = Vec::with_capacity(height);
    for y in range(0u, height) {
        let line = render_line(width, height, y, nsubsamples, objects);
        for x in range(0u, width) {
            lines.push(line[x]);
        }
    }
    return lines;
}

/*
type RenderedPixels = (uint, [Pixel]);

struct RenderTask {
    task_id: uint,
    width: uint,
    height: uint,
    nsubsamples: uint,
    objects: std::arc::ARC<[Object]>,
    sender: Chan<RenderedPixels>,
    receiver: Port<uint>
}

fn render_multitask_run(rt: RenderTask) {
    println!(" Task {}: started.", rt.task_id);
    let objects = copy *std::arc::get(&rt.objects);
    loop {
        let line = rt.receiver.recv();
        if line >= rt.height {
            break;
        }
        //println!(" Task {}: start rendering: line = {}", rt.task_id, line);
        let pixels = render_line(rt.width, rt.height, line, rt.nsubsamples, objects);
        rt.sender.send((line, pixels));
    }
    println!(" Task {}: finished.", rt.task_id);
}

fn render_multitask(width: uint, height: uint, nsubsamples: uint,
                    num_task: uint, objects: [Object]) -> [Pixel] {
    let objects_arc = std::arc::ARC(objects);
    let mut receivers: [Port<RenderedPixels>] = [];
    let mut senders: [Chan<uint>] = [];

    println!("Spawn {} tasks.", num_task);
    for i in uint::range(0u, num_task) {
        let (pixel_receiver, pixel_sender):
            (Port<RenderedPixels>, Chan<RenderedPixels>) = comm::stream();
        let (line_receiver, line_sender):
            (Port<uint>, Chan<uint>) = comm::stream();
        let rt = RenderTask {
            task_id: i,
            width: width,
            height: height,
            nsubsamples: nsubsamples,
            objects: objects_arc.clone(),
            sender: pixel_sender,
            receiver: line_receiver
        };
        receivers.push(pixel_receiver);
        senders.push(line_sender);
        task::spawn_with(rt, render_multitask_run);
    }

    struct RenderResult { pixels: [Pixel] };
    let mut result_store:[RenderResult] = vec::with_capacity(height);
    for _ in uint::range(0u, height) {
        result_store.push(RenderResult{ pixels: [] });
    }
    // start render
    let mut rendered_line = 0u;
    let mut assigned_line = 0u;
    for i in uint::range(0u, num_task) {
        senders[i].send(assigned_line);
        assigned_line += 1;
    }
    while rendered_line < height {
        for i in uint::range(0u, num_task) {
            match receivers[i].try_recv() {
                Some((line, pixels)) => {
                    //println!(" line {} received from Task {}", line, i);
                    result_store[line].pixels = pixels;
                    rendered_line += 1;
                    if rendered_line < height {
                        senders[i].send(assigned_line);
                        assigned_line += 1;
                    } else {
                        senders[i].send(height); // meaning end task
                    }
                },
                None => {
                    // rendering now
                }
            }
        }
    }
    // merge result
    let mut pixels: [Pixel] = [];
    for t in result_store.each() { pixels.push_all(t.pixels); }
    return pixels;
}
*/

fn saveppm(filename: &str, width: uint, height: uint, pixels: Vec<Pixel>) {
    let file = File::create(&Path::new(filename));
    let mut writer = BufferedWriter::new(file);
    //writer.write_str(format!("P6\n{0} {1}\n255\n", width, height));
    writer.write_str("P6\n256 256\n255\n");
    for pixel in pixels.iter() {
        writer.write([pixel.r, pixel.g, pixel.b]);
    };
}

fn main() {
    let objects = [Sphere(vector::new(-2.0, 0.0, -3.5), 0.5),
                   Sphere(vector::new(-0.5, 0.0, -3.0), 0.5),
                   Sphere(vector::new( 1.0, 0.0, -2.2), 0.5),
                   Plane(vector::new(0.0, -0.5, 0.0), vector::new(0.0, 1.0, 0.0))];
    let width = 256u;
    let height = 256u;
    let mut num_task = 1u;
    //let args = os::args();
    let mut pixels;
    /*
    if args.len() >= 2u {
        num_task = from_str(args[1]).get();
    }
    if num_task == 1 {
        pixels = render_singletask(width, height, NSUBSAMPLES, objects);
    } else {
        pixels = render_singletask(width, height, NSUBSAMPLES, objects);
        //pixels = render_multitask(width, height, NSUBSAMPLES, num_task, objects);
    }*/
    pixels = render_singletask(width, height, NSUBSAMPLES, objects);
    saveppm("image.ppm", width, height, pixels);
}


