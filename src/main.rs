/* -*- Mode: rust; tab-width: 4; c-basic-offset: 4; indent-tabs-mode: nil; -*- */

#![allow(unused_parens)]

extern crate rand;

use std::cmp;

static NAO_SAMPLES: u32 = 8;
static NSUBSAMPLES: u32 = 2;

mod vector3 {
    use std::ops::{Add, Sub};
    
    pub struct Vector3 {
        pub x: f32,
        pub y: f32,
        pub z: f32
    }

    pub fn new(x: f32, y: f32, z: f32) -> Vector3 {
        Vector3 { x: x, y: y, z: z }
    }

    pub fn new_normal(x: f32, y: f32, z: f32) -> Vector3 {
        let mut v = Vector3 { x: x, y: y, z: z };
        v.normalized();
        return v;
    }

    // operator +
    impl Add for Vector3 {
        type Output = Vector3;
        fn add(self, other: Vector3) -> Vector3 {
            Vector3 { x: self.x + other.x,
                      y: self.y + other.y,
                      z: self.z + other.z }
        }
    }

    // operator -
    impl Sub for Vector3 {
        type Output = Vector3;
        fn sub(self, other: Vector3) -> Vector3 {
            Vector3 { x: self.x - other.x,
                      y: self.y - other.y,
                      z: self.z - other.z }
        }
    }

    pub fn dot(v0: &Vector3, v1: &Vector3) -> f32 {
        v0.x * v1.x + v0.y * v1.y + v0.z * v1.z
    }

    pub fn cross(v0: &Vector3, v1: &Vector3) -> Vector3 {
        Vector3 { x: v0.y * v1.z - v0.z * v1.y,
                  y: v0.z * v1.x - v0.x * v1.z,
                  z: v0.x * v1.y - v0.y * v1.x }
    }

    pub fn scale(v: &Vector3, s: f32) -> Vector3 {
        Vector3 { x: v.x * s, y: v.y * s, z: v.z * s }
    }

    impl Vector3 {
        pub fn normalized(&mut self) {
            let length = dot(self, self).sqrt();
            if (length < -1.0e-9) || (length > 1.0e-9) {
                self.x /= length;
                self.y /= length;
                self.z /= length;
            }
        }
    }

    impl Clone for Vector3 {
        fn clone(&self) -> Self {
            Vector3 { x: self.x, y: self.y, z: self.z }
        }
        fn clone_from(&mut self, source: &Self) {
            self.x = source.x;
            self.y = source.y;
            self.z = source.z;
        }
    }

    impl Copy for Vector3 { }
}

struct Ray {
    origin: vector3::Vector3,
    direction: vector3::Vector3
}

struct IntersectInfo {
    distance: f32,
    position: vector3::Vector3,
    normal: vector3::Vector3
}

enum Object {
    Sphere(vector3::Vector3, f32),
    Plane(vector3::Vector3, vector3::Vector3)
}

#[allow(non_snake_case)]
impl Object {
    pub fn intersect(&self, ray: &Ray, isect: &mut IntersectInfo) -> bool {
        match self {
            &Object::Sphere(ref position, radius) => {
                let rs = ray.origin - *position;
                let B = vector3::dot(&rs, &ray.direction);
                let C = vector3::dot(&rs, &rs) - radius * radius;
                let D = B * B - C;
                if D > 0.0 {
                    let t = -B - D.sqrt();
                    if (t > 0.0) && (t < isect.distance) {
                        isect.distance = t;
                        isect.position = ray.origin + vector3::scale(&ray.direction, t);
                        isect.normal = isect.position - *position;
                        isect.normal.normalized();
                        return true;
                    }
                }
                return false;
            },
            &Object::Plane(ref position, ref normal) => {
                let d = -vector3::dot(&position, &normal);
                let v = vector3::dot(&ray.direction, &normal);
                if v.abs() < 1.0e-9f32 { return false; }
                let t = -(vector3::dot(&ray.origin, &normal) + d) / v;
                if (t > 0.0) && (t < isect.distance) {
                    isect.distance = t;
                    isect.position = ray.origin + vector3::scale(&ray.direction, t);
                    isect.normal = *normal;
                    return true;
                }
                return false;
            }
        }
    }
}

#[inline]
fn ortho_basis(n: vector3::Vector3) -> [vector3::Vector3; 3] {
    let basis1 =
        if (n.x < 0.6) && (n.x > -0.6) {
            vector3::new(1.0, 0.0, 0.0)
        } else if ((n.y < 0.6) && (n.y > -0.6)) {
            vector3::new(0.0, 1.0, 0.0)
        } else if ((n.z < 0.6) && (n.z > -0.6)) {
            vector3::new(0.0, 0.0, 1.0)
        } else {
            vector3::new(1.0, 0.0, 0.0)
        };
    let mut basis0 = vector3::cross(&basis1, &n);
    basis0.normalized();

    let mut basis1 = vector3::cross(&n, &basis0);
    basis1.normalized();

    return [basis0, basis1, n];
}

#[allow(deprecated)]
fn ambient_occlusion(isect: &IntersectInfo,
                     objects: &[Object]) -> f32 {
    let eps = 0.0001f32;
    let ntheta = NAO_SAMPLES;
    let nphi = NAO_SAMPLES;
    let ray_origin = isect.position + vector3::scale(&isect.normal, eps);
    let basis = ortho_basis(isect.normal);
    let mut occlusion: f32 = 0.0;
    let mut occ_isect = IntersectInfo {
        distance: 1.0e+9,
        position: vector3::new(0.0, 0.0, 0.0),
        normal: vector3::new(0.0, 1.0, 0.0)
    };
    let tau: f32 = std::f32::consts::PI * 2.0;

    for _ in 0..ntheta {
        for _ in 0..nphi {
            let theta = rand::random::<f32>().sqrt();
            let phi = tau * rand::random::<f32>();

            let x = phi.cos() * theta;
            let y = phi.sin() * theta;
            let z = (1.0 - theta * theta).sqrt();

            // local -> global
            let direction = vector3::Vector3 {
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
    #[inline]
    pub fn new(r:u8, g:u8, b:u8) -> Pixel {
        Pixel { r:r, g:g, b:b }
    }
    */

    #[inline]
    pub fn new_with_clamp(value: f32, mag: f32) -> Pixel {
        let v = (value * mag) as u32;
        let i = cmp::min(255, v) as u8;
        return Pixel { r:i, g:i, b:i };
    }
}

fn render(width: u32, height: u32,
          nsubsamples: u32, objects: &[Object]) -> Vec<Pixel> {
    let w = width as f32;
    let h = height as f32;
    let cx = w / 2.0;
    let cy = h / 2.0;
    let mut pixels: Vec<Pixel> = Vec::with_capacity((width * height) as usize);
    let sample: f32 = nsubsamples as f32;
    for y in 0u32..height {
        for x in 0u32..width {
            let mut occlusion = 0.0f32;
            for u in 0u32..nsubsamples {
                for v in 0u32..nsubsamples {
                    let x = x as f32;
                    let y = y as f32;
                    let u = u as f32;
                    let v = v as f32;
                    let px =  (x + (u / sample) - cx) / cx;
                    let py = -(y + (v / sample) - cy) / cy;
                    let ray = Ray { origin: vector3::new(0.0, 0.0, 0.0),
                                    direction: vector3::new_normal(px, py, -1.0) };
                    let mut isect = IntersectInfo {
                        distance: 1.0e+17,
                        position: vector3::new(0.0, 0.0, 0.0),
                        normal: vector3::new(0.0, 1.0, 0.0)
                    };
                    let mut hit = false;
                    for o in objects.iter() {
                        hit = (hit || o.intersect(&ray, &mut isect));
                    }
                    occlusion += if hit { ambient_occlusion(&mut isect, objects) } else { 0.0 };
                }
            }
            if occlusion > 0.0001 {
                let c = occlusion / ((nsubsamples * nsubsamples) as f32);
                pixels.push(Pixel::new_with_clamp(c, 255.0));
            } else {
                pixels.push(Pixel { r:0u8, g:0u8, b:0u8 });
            }
        }
    }
    return pixels;
}

#[allow(unused_must_use)]
fn saveppm(filename: &str, width: u32, height: u32, pixels: Vec<Pixel>) {
    use std::io::prelude::*;
    use std::fs::File;
    use std::io::BufWriter;

    let f = File::create(filename).unwrap();
    let mut w = BufWriter::new(f);
    let head = format!("P6\n{0} {1}\n255\n", width, height);
    w.write_all(head.as_bytes());
    for pixel in pixels.iter() {
        w.write(&[pixel.r, pixel.g, pixel.b]);
    };
}

fn main() {
    let objects = [Object::Sphere(vector3::new(-2.0, 0.0, -3.5), 0.5),
                   Object::Sphere(vector3::new(-0.5, 0.0, -3.0), 0.5),
                   Object::Sphere(vector3::new( 1.0, 0.0, -2.2), 0.5),
                   Object::Plane(vector3::new(0.0, -0.5, 0.0), vector3::new(0.0, 1.0, 0.0))];
    let width = 256u32;
    let height = 256u32;
    let pixels = render(width, height, NSUBSAMPLES, &objects);
    saveppm("image.ppm", width, height, pixels);
}


