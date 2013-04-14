/* -*- Mode: c; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil; -*- */

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
            if (length < -1.0e-9) || (length > 1.0e-9) {
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
                return false;
            },
            Plane(position, normal) => {
                let d = -vector::dot(&position, &normal);
                let v = vector::dot(&ray.direction, &normal);
                if abs(v) < 1.0e-9 { return false; }
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
                     rng: @rand::Rng,
                     objects: &[Object]) -> float {
    let eps = 0.0001;
    let ntheta = NAO_SAMPLES;
    let nphi = NAO_SAMPLES;
    let mut ray_origin = isect.position + vector::scale(&isect.normal, eps);
    let basis = ortho_basis(isect.normal);
    let mut occlusion: float = 0.0;
    let mut occ_isect = ~IntersectInfo {
        distance: 1.0e+9,
        position: vector::new(0.0, 0.0, 0.0),
        normal: vector::new(0.0, 1.0, 0.0)
    };
    let tau: float = 2.0f * float::consts::pi;

    for uint::range(0u, ntheta) |_| {
        for uint::range(0u, nphi) |_| {
            let theta = sqrt(rng.gen_float());
            let phi = tau * rng.gen_float();

            let x = float::cos(phi) * theta;
            let y = float::sin(phi) * theta;
            let z = float::sqrt(1.0f - theta * theta);

            // local -> global
            let direction = vector::Vector {
                x: x * basis[0].x + y * basis[1].x + z * basis[2].x,
                y: x * basis[0].y + y * basis[1].y + z * basis[2].y,
                z: x * basis[0].z + y * basis[1].z + z * basis[2].z
            };
            let ray = Ray { origin: ray_origin,
                            direction: direction };
            occ_isect.distance = 1.0e+9;
            for objects.each |o| {
                if o.intersect(&ray, occ_isect) {
                    occlusion += 1.0;
                    break;
                }
            }
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
        let i = uint::min(255u, v) as u8;
        return Pixel { r:i, g:i, b:i };
    }
}

fn render_line(width: uint, height: uint, _y: uint,
               nsubsamples: uint, objects: &[Object]) -> ~[Pixel] {
    let mut line = vec::with_capacity(width);
    let sample: float = nsubsamples as float;
    let rng = rand::Rng();
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
                    occlusion += ambient_occlusion(isect, rng, objects);
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


type RenderedPixels = (uint, ~[Pixel]);

struct RenderTask {
    task_id: uint,
    width: uint,
    height: uint,
    nsubsamples: uint,
    objects: std::arc::ARC<~[Object]>,
    sender: Chan<RenderedPixels>,
    receiver: Port<uint>
}

fn render_multitask_run(rt: ~RenderTask) {
    println(fmt!(" Task %u: started.", rt.task_id));
    let objects = copy *std::arc::get(&rt.objects);
    loop {
        let line = rt.receiver.recv();
        if line >= rt.height {
            break;
        }
        //println(fmt!(" Task %u: start rendering: line = %u", rt.task_id, line));
        let pixels = render_line(rt.width, rt.height, line, rt.nsubsamples, objects);
        rt.sender.send((line, pixels));
    }
    println(fmt!(" Task %u: finished.", rt.task_id));
}

fn render_multitask(width: uint, height: uint, nsubsamples: uint,
                    num_task: uint, objects: ~[Object]) -> ~[Pixel] {
    let objects_arc = std::arc::ARC(objects);
    let mut receivers: ~[Port<RenderedPixels>] = ~[];
    let mut senders: ~[Chan<uint>] = ~[];

    println(fmt!("Spawn %u tasks.", num_task));
    for uint::range(0u, num_task) |i| {
        let (pixel_receiver, pixel_sender):
            (Port<RenderedPixels>, Chan<RenderedPixels>) = comm::stream();
        let (line_receiver, line_sender):
            (Port<uint>, Chan<uint>) = comm::stream();
        let rt = ~RenderTask {
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

    struct RenderResult { pixels: ~[Pixel] };
    let mut result_store:~[RenderResult] = vec::with_capacity(height);
    for uint::range(0u, height) |_| {
        result_store.push(RenderResult{ pixels: ~[] });
    }
    // start render
    let mut rendered_line = 0u;
    let mut assigned_line = 0u;
    for uint::range(0u, num_task) |i| {
        senders[i].send(assigned_line);
        assigned_line += 1;
    }
    while rendered_line < height {
        for uint::range(0u, num_task) |i| {
            match receivers[i].try_recv() {
                Some((line, pixels)) => {
                    //println(fmt!(" line %u received from Task %u", line, i));
                    result_store[line].pixels = pixels;
                    rendered_line += 1;
                    if rendered_line < height {
                        senders[i].send(assigned_line);
                        assigned_line += 1;
                    } else {
                        senders[i].send(height); // meaning end task
                    }
                }
                None => {
                    /* rendering now */
                }
            }
        }
    }
    /* merge result */
    let mut pixels: ~[Pixel] = ~[];
    for result_store.each() |t| { pixels.push_all(t.pixels); }
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
        num_task = uint::from_str(args[1]).get();
    }
    if num_task == 1 {
        pixels = render_singletask(width, height, NSUBSAMPLES, objects);
    } else {
        pixels = render_multitask(width, height, NSUBSAMPLES, num_task, objects);
    }
    saveppm("image.ppm", width, height, pixels);
}


