/* -*- Mode: js; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil; -*- */

use core::float::{sqrt, abs};
use core::rand::RngUtil;
use core::io::{Writer, WriterUtil};

use core::comm::{Port, Chan};
use std;

static NAO_SAMPLES: uint = 8;
static NSUBSAMPLES: uint = 2;

mod vector {
    use core::float::sqrt;

    pub struct Vector {
        x: float,
        y: float,
        z: float
    }

    #[inline(always)]
    pub fn new(x: float, y: float, z: float) -> Vector {
        Vector { x: x, y: y, z: z }
    }

    #[inline(always)]
    pub fn new_normal(x: float, y: float, z: float) -> Vector {
        let mut v = Vector { x: x, y: y, z: z };
        v.normalized();
        return v;
    }

    // operator +
    #[inline(always)]
    impl ops::Add<Vector, Vector> for Vector {
        fn add(&self, other: &Vector) -> Vector {
            Vector { x: self.x + other.x,
                     y: self.y + other.y,
                     z: self.z + other.z }
        }
    }

    // operator -
    #[inline(always)]
    impl ops::Sub<Vector, Vector> for Vector {
        fn sub(&self, other: &Vector) -> Vector {
            Vector { x: self.x - other.x,
                     y: self.y - other.y,
                     z: self.z - other.z }
        }
    }

    // operator *
    #[inline(always)]
    impl ops::Mul<Vector, Vector> for Vector {
        fn mul(&self, other: &Vector) -> Vector {
            Vector { x: self.x * other.x,
                     y: self.y * other.y,
                     z: self.z * other.z }
        }
    }

    #[inline(always)]
    pub fn dot(v0: &Vector, v1: &Vector) -> float {
        v0.x * v1.x + v0.y * v1.y + v0.z * v1.z
    }

    #[inline(always)]
    pub fn cross(v0: &Vector, v1: &Vector) -> Vector {
        Vector { x: v0.y * v1.z - v0.z * v1.y,
                 y: v0.z * v1.x - v0.x * v1.z,
                 z: v0.x * v1.y - v0.y * v1.x }
    }

    #[inline(always)]
    pub fn scale(v: &Vector, s: float) -> Vector {
        Vector { x: v.x * s, y: v.y * s, z: v.z * s }
    }

    pub impl Vector {
        #[inline(always)]
        pub fn normalized(&mut self) {
            let length = sqrt(dot(self, self));
            if (length < -1.0e-17) || (length > 1.0e-17) {
                self.x /= length;
                self.y /= length;
                self.z /= length;
            }
        }
    }
}

struct Ray {
    origin: vector::Vector,
    direction: vector::Vector
}

struct IntersectInfo {
    distance: float,
    position: vector::Vector,
    normal: vector::Vector
}

enum Object {
    Sphere(vector::Vector, float),
    Plane(vector::Vector, vector::Vector)
}

impl Object {
    pub fn intersect(&self, ray: &Ray, isect: &mut IntersectInfo) -> bool {
        match *self {
            Sphere(position, radius) => {
                let rs = ray.origin - position;
                let B = vector::dot(&rs, &ray.direction);
                let C = vector::dot(&rs, &rs) - radius * radius;
                let D = B * B - C;
                if D > 0.0 {
                    let t = -B - sqrt(D);
                    if (t > 0.0) && (t < isect.distance) {
                        isect.distance = t;
                        isect.position = ray.origin + vector::scale(&ray.direction, t);
                        isect.normal = isect.position - position;
                        isect.normal.normalized();
                        return true;
                    }
                }
            },
            Plane(position, normal) => {
                let d = -vector::dot(&position, &normal);
                let v = vector::dot(&ray.direction, &normal);
                if abs(v) < 1.0e-17 { return false; }
                let t = -(vector::dot(&ray.origin, &normal) + d) / v;
                if (t > 0.0) && (t < isect.distance) {
                    isect.distance = t;
                    isect.position = ray.origin + vector::scale(&ray.direction, t);
                    isect.normal = normal;
                    return true;
                }
            }
        }
        return false;
    }
}

// ---

#[inline(always)]
fn ortho_basis(n: vector::Vector) -> [vector::Vector, ..3] {
    // 'if' is not statement. it's expression.
    let mut basis1 =
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
                     objects: &[Object]) -> float {
    let eps = 0.0001;
    let ntheta = NAO_SAMPLES;
    let nphi = NAO_SAMPLES;
    let mut ray_origin = isect.position + vector::scale(&isect.normal, eps);
    let basis = ortho_basis(isect.normal);
    let rng = rand::Rng();
    let mut occlusion: float = 0.0;
    let mut occ_isect = ~IntersectInfo {
        distance: 1.0e+9,
        position: vector::new(0.0, 0.0, 0.0),
        normal: vector::new(0.0, 1.0, 0.0)
    };

    for uint::range(0u, ntheta) |_| {
        for uint::range(0u, nphi) |_| {
            let theta = sqrt(rng.gen_float());
            let phi = 2.0f * float::consts::pi * rng.gen_float();

            let x = float::cos(phi) * theta;
            let y = float::sin(phi) * theta;
            let z = float::sqrt(1.0f - theta * theta);

            // local -> global
            let global = vector::Vector {
                x: x * basis[0].x + y * basis[1].x + z * basis[2].x,
                y: x * basis[0].y + y * basis[1].y + z * basis[2].y,
                z: x * basis[0].z + y * basis[1].z + z * basis[2].z
            };
            let ray = Ray { origin: ray_origin,
                            direction: global };
            occ_isect.distance = 1.0e+9;
            let mut hit = false;
            for objects.each |o| {
                let h = o.intersect(&ray, occ_isect);
                hit = (hit || h);
            }
            if hit { occlusion += 1.0; }
        }
    }
    let theta = ntheta as float;
    let phi = nphi as float;
    return (theta * phi - occlusion) / (theta * phi);
}


struct Pixel {
    r:u8, g:u8, b:u8
}

impl Pixel {
    #[inline(always)]
    pub fn new(r:u8, g:u8, b:u8) -> Pixel {
        Pixel { r:r, g:g, b:b }
    }

    #[inline(always)]
    pub fn new_with_clamp(value: float, mag: float) -> Pixel {
        let v = (value * mag) as uint;
        let i = match v {
            0..255 => v as u8,
            _      => if v > 255 { 255u8 } else { 0u8 }
        };
        return Pixel { r:i, g:i, b:i };
    }
}

fn render_line(width: uint, height: uint, _y: uint,
               nsubsamples: uint, objects: &[Object]) -> ~[Pixel] {
    let mut line = vec::with_capacity(width);
    let sample: float = nsubsamples as float;
    let w: float = width as float;
    let h: float = height as float;
    let y: float = _y as float;
    for uint::range(0u, width) |_x| {
        let mut occlusion = 0.0f;
        for uint::range(0u, nsubsamples) |_u| {
            for uint::range(0u, nsubsamples) |_v| {
                let x: float = _x as float;
                let u: float = _u as float;
                let v: float = _v as float;
                let px: float = (x + (u / sample) - (w / 2.0f)) / (w / 2.0f);
                let py: float = -(y + (v / sample) - (h / 2.0f)) / (h / 2.0f);
                let ray = Ray { origin: vector::new(0.0, 0.0, 0.0),
                                direction: vector::new_normal(px, py, -1.0) };

                let mut isect = ~IntersectInfo {
                    distance: 1.0e+17,
                    position: vector::new(0.0, 0.0, 0.0),
                    normal: vector::new(0.0, 1.0, 0.0)
                };
                let mut hit = false;
                for objects.each |o| {
                    let h = o.intersect(&ray, isect);
                    hit = (hit || h);
                }
                if hit {
                    occlusion += ambient_occlusion(isect, objects);
                }
            }
        }
        if occlusion > 0.0001 {
            let c = occlusion / ((nsubsamples * nsubsamples) as float);
            line.push(Pixel::new_with_clamp(c, 255.0));
        } else {
            line.push(Pixel { r:0u8, g:0u8, b:0u8 });
        }
    }
    return line;
}

fn render_singletask(width: uint, height: uint, nsubsamples: uint, objects: ~[Object]) -> ~[Pixel] {
    let mut lines = vec::with_capacity(height);
    for uint::range(0u, height) |y| {
        let line = render_line(width, height, y, nsubsamples, objects);
        lines.push(line);
    }
    return vec::concat(lines);
}

struct RenderTask {
    task_id: uint,
    width: uint,
    height: uint,
    run_y0: uint,
    run_y1: uint,
    nsubsamples: uint,
    objects: std::arc::ARC<~[Object]>,
    channel: Chan<~[Pixel]>
}

fn render_multitask_run(rt: ~RenderTask) {
    println(fmt!(" Task %u started (%u - %u).", rt.task_id, rt.run_y0, rt.run_y1));
    let mut lines = vec::with_capacity((rt.run_y1 - rt.run_y0) * rt.width);
    let objects = copy *std::arc::get(&rt.objects);
    for uint::range(rt.run_y0, rt.run_y1) |y| {
        let line = render_line(rt.width, rt.height, y, rt.nsubsamples, objects);
        lines.push(line);
    }
    println(fmt!(" Task %u finished (%u - %u).", rt.task_id, rt.run_y0, rt.run_y1));
    rt.channel.send(vec::concat(lines));
}

fn render_multitask(width: uint, height: uint, nsubsamples: uint,
                    num_task: uint, objects: ~[Object]) -> ~[Pixel] {
    let run_height = height / num_task;
    let mut run = 0u;
    let objects_arc = std::arc::ARC(objects);
    let mut ports: ~[Port<~[Pixel]>] = ~[];

    println(fmt!("Spawn %u tasks.", num_task));
    for uint::range(0u, num_task) |i| {
        let (port, chan): (Port<~[Pixel]>, Chan<~[Pixel]>) = comm::stream();
        let rt = ~RenderTask {
            task_id: i,
            width: width,
            height: height,
            run_y0: run,
            run_y1: if i == (num_task-1) {height} else { run + run_height },
            nsubsamples: nsubsamples,
            objects: objects_arc.clone(),
            channel: chan
        };
        task::spawn_with(rt, render_multitask_run);
        ports.push(port);
        run += run_height;
    }

    let mut pixels: ~[Pixel] = ~[];

    for ports.each() |p| {
        let chunk = p.recv();
        io::println(fmt!(" Received chunk, size: %? bytes", chunk.len()));
        pixels.push_all(chunk);
    }

    //let mut lines = vec::with_capacity(height);
    return pixels;
}

fn saveppm(filename: &str, width: uint, height: uint, pixels: &[Pixel]) {
    let writer = result::get(&io::buffered_file_writer(&Path(filename)));
    writer.write_str(fmt!("P6\n%u %u\n255\n", width, height));
    for pixels.each |pixel| {
        writer.write([pixel.r, pixel.g, pixel.b]);
    };
}

fn main() {
    let objects = ~[Sphere(vector::new(-2.0, 0.0, -3.5), 0.5),
                    Sphere(vector::new(-0.5, 0.0, -3.0), 0.5),
                    Sphere(vector::new( 1.0, 0.0, -2.2), 0.5),
                    Plane(vector::new(0.0, -0.5, 0.0), vector::new(0.0, 1.0, 0.0))];
    let width = 256u;
    let height = 256u;
    let mut num_task = 1u;
    let args = os::args();
    let mut pixels;
    if args.len() >= 2u {
        println(fmt!("%?", uint::from_str(args[1])));
        num_task = uint::from_str(args[1]).get();
    }
    if num_task == 1 {
        pixels = render_singletask(width, height, NSUBSAMPLES, objects);
    } else {
        pixels = render_multitask(width, height, NSUBSAMPLES, num_task, objects);
    }
    saveppm("image.ppm", width, height, pixels);
}


