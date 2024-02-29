use std::{
    f32::{
        consts::{PI, SQRT_2},
        NAN,
    },
    ops::{Add, Sub},
    thread::sleep,
    time::Duration,
};

use ncurses::{
    addch, attroff, attrset, clear, curs_set, endwin, getch, getmaxx, getmaxy, init_pair, initscr,
    noecho, refresh, start_color, stdscr, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_PAIR,
    COLOR_RED,
};

const STEP_SIZE: f32 = 1.0;
const X_INIT_POS: f32 = 10.0;
const SUMO_SIZE: f32 = 2.5;
const COLLISION_BOUNDRY_SIZE: f32 = 0.0;
const DRAWING_BOUNDRY_SIZE: f32 = 0.5;
const ORIGIN: Vec2 = Vec2 { x: 0.0, y: 0.0 };
const REQ_CASTER: SumoState = SumoState { center: ORIGIN, dir: 0.0, corners: [ORIGIN, ORIGIN, ORIGIN, ORIGIN]};
const TATAMI_SIZE: f32 = 20.0;
const PUSH_FRICTION: f32 = 1.0;
const M: f32 = SUMO_SIZE * 4.0 * (SQRT_2 - 1.0) / PI;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    x: f32,
    y: f32,
}

impl Vec2 {
    fn round(self) -> Vec2 {
        Vec2 {
            x: self.x.round(),
            y: self.y.round()
        }
    }
    fn origin_dir(self) -> f32 {
        f32::atan(self.x / self.y)
    }

    fn dist(self, sstate: Self) -> f32 {
        let vstate = self - sstate;
        f32::sqrt(vstate.x * vstate.x + vstate.y * vstate.y)
    }
}

impl Add<Vec2> for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub<Vec2> for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Add<Vec2> for SumoState {
    type Output = SumoState;
    fn add(self, rhs: Vec2) -> Self::Output {
        SumoState {
            center: rhs + self.center,
            dir: self.dir,
            corners: self.corners.map(|corner| corner + rhs),
        }
    }
}

type Corners = [Vec2; 4];
#[derive(Clone, Copy, Debug)]
pub struct SumoState {
    center: Vec2,
    dir: f32,
    corners: Corners,
}

impl SumoState {
    fn radius_towards(self, theta: f32) -> f32 {
        let rel_dir = (theta - self.dir) % (PI / 2.0);
        if rel_dir <= PI / 4.0 {
            M * rel_dir + SUMO_SIZE
        } else {
            M * (PI / 2.0 - rel_dir) + SUMO_SIZE
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Robocillo {
    Paco,
    Curro,
}

#[derive(Clone, Copy)]
struct SumoReq {
    motor_l: f32,
    motor_r: f32,
}
impl SumoReq {
    fn vel(self) -> f32 {
        (self.motor_l + self.motor_r) / 2.0
    }
}

// impl Into<SumoState> for SumoReq {
//     fn into(self) -> SumoState {
//         let theta_p =
//             (STEP_SIZE * (self.motor_r - self.motor_l) / SUMO_SIZE) % (2.0 * PI);
//         let vel = self.vel();
//         SumoState {
//             center: Vec2 {
//                 x: theta_p.cos() * vel,
//                 y: theta_p.sin() * vel
//             },
//             dir: theta_p,
//         }
//     }
// }
//
impl Add<SumoReq> for SumoState {
    type Output = SumoState;
    fn add(self, sstate: SumoReq) -> SumoState {
        let theta_p =
            ((STEP_SIZE * (sstate.motor_r - sstate.motor_l) / SUMO_SIZE) + self.dir) % (2.0 * PI);
        let vel = sstate.vel();

        let dv = Vec2 {
            x: theta_p.cos() * vel,
            y: theta_p.sin() * vel,
        };
        SumoState {
            center: self.center + dv,
            dir: theta_p,
            corners: self.corners.map(|corner| corner + dv),
        }
    }
}

fn is_near<T>(x: T, y: T, bound: T) -> bool
where
    T: Add<Output = T>,
    T: Sub<Output = T>,
    T: PartialOrd,
    T: Copy,
{
    x < y + bound && x > y - bound
}

type Strategy = fn(f32) -> (f32, f32);
pub fn probe_strategy(strat1: Strategy, strat2: Strategy) -> Vec<[SumoState; 2]> {
    let mut sym_state = [
        SumoState {
            center: Vec2 {
                x: X_INIT_POS,
                y: 0.0,
            },
            dir: PI,
            corners: [
                Vec2 {
                    x: X_INIT_POS + SUMO_SIZE / 2.0,
                    y: SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: X_INIT_POS + SUMO_SIZE / 2.0,
                    y: -SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: X_INIT_POS - SUMO_SIZE / 2.0,
                    y: SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: X_INIT_POS - SUMO_SIZE / 2.0,
                    y: -SUMO_SIZE / 2.0,
                },
            ],
        },
        SumoState {
            center: Vec2 {
                x: -X_INIT_POS,
                y: 0.0,
            },
            dir: 0.0,
            corners: [
                Vec2 {
                    x: (-X_INIT_POS) + SUMO_SIZE / 2.0,
                    y: SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: (-X_INIT_POS) + SUMO_SIZE / 2.0,
                    y: -SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: (-X_INIT_POS) - SUMO_SIZE / 2.0,
                    y: SUMO_SIZE / 2.0,
                },
                Vec2 {
                    x: (-X_INIT_POS) - SUMO_SIZE / 2.0,
                    y: -SUMO_SIZE / 2.0,
                },
            ],
        },
    ];

    fn as_req((motor_l, motor_r): (f32, f32)) -> SumoReq {
        SumoReq { motor_l, motor_r }
    }

    fn update(
        [sysl, sysr]: [SumoState; 2],
        [reql, reqr]: [SumoReq; 2],
    ) -> Result<[SumoState; 2], Robocillo> {
        let mut sy_s = [sysl + reql, sysr + reqr];
        let theta = (sy_s[0].center - sy_s[1].center).origin_dir();
        if sysl.center.dist(sysr.center)
            < dbg!(sy_s[0].radius_towards(theta))
                + sy_s[1].radius_towards(theta)
                + COLLISION_BOUNDRY_SIZE
        {
            let vatt = reqr.vel() - reql.vel();
            let ap = if vatt > 0.0 {
                |x, y| x + y
            } else {
                |x, y| x + (ORIGIN - y)
            };
            let (gyatt, xatt) = (REQ_CASTER + reqr).dir.sin_cos(); dbg!("hi");
            let vec_push = Vec2 {
                x: xatt * vatt / PUSH_FRICTION,
                y: gyatt * vatt / PUSH_FRICTION,
            };
            sy_s = [ap(sysl, vec_push), ap(sysr, ORIGIN - vec_push)];
        };
        if sysl.center.dist(ORIGIN) < TATAMI_SIZE {
            if sysr.center.dist(ORIGIN) < TATAMI_SIZE {
                Ok(sy_s)
            } else {
                Err(Robocillo::Paco)
            }
        } else {
            Err(Robocillo::Curro)
        }
    }

    fn calc_ir(s1: SumoState, s2: SumoState, dist: f32) -> f32 {
        let (min_corner, max_corner) = s2
            .corners
            .iter()
            .map(|v| v.origin_dir())
            .fold((0.0f32, 0.0f32), |(x, y), t| (x.min(y), y.max(t)));
        if s1.dir <= max_corner && s1.dir >= min_corner {
            dist
        } else {
            0.0
        }
    }

    let mut states = vec![sym_state];
    loop {
        let dist = sym_state[0].center.dist(sym_state[1].center);
        let ir_reads: [SumoReq; 2] = [
            as_req(strat1(calc_ir(sym_state[0], sym_state[1], dist))),
            as_req(strat2(calc_ir(sym_state[1], sym_state[0], dist))),
        ];
        match update(sym_state, ir_reads) {
            Ok(symst) => sym_state = symst,
            Err(_) => break,
        };
        states.push(sym_state);
    }
    states
}

pub fn graphics_driver(states: Vec<[SumoState; 2]>) {
    let maxx = getmaxx(stdscr());
    let maxy = getmaxy(stdscr());

    fn as_linear(x: f32, s: SumoState) -> f32 {
        let dom_res = if s.dir > PI / 2.0 && s.dir < PI * 1.5 {
            |x, y| x < y
        } else {
            |x, y| x > y
        };
        if dom_res(s.center.x, x) {
            return NAN;
        }
        let m = s.dir.tan();
        x * m + (s.center.y - s.center.x * m)
    }

    for frame in 0..states.len() {
        clear();
        for y in 0..maxy {
            for x in 0..maxx - 1 {
                let cx = (x / 2 - maxx / 4) as f32;
                let cy = (y - maxy / 2) as f32;
                let d = f32::sqrt((cx * cx) as f32 + (cy * cy) as f32);
                if states[frame][0].center.x.round() == cx
                    && states[frame][0].center.y.round() == cy
                {
                    attrset(COLOR_PAIR(1));
                    addch('=' as u32);
                } else if states[frame][1].center.x.round() == cx
                    && states[frame][1].center.y.round() == cy
                {
                    attrset(COLOR_PAIR(2));
                    addch('=' as u32);
                } else if states[frame][0].corners.map(|corner| corner.round()).contains(&Vec2 { x: cx, y: cy }) {
                    attrset(COLOR_PAIR(1));
                    addch('o' as u32);
                } else if states[frame][1].corners.map(|corner| corner.round()).contains(&Vec2 { x: cx, y: cy }) {
                    attrset(COLOR_PAIR(2));
                    addch('o' as u32);
                } else if is_near(d, TATAMI_SIZE as f32, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(0));
                    addch('#' as u32);
                } else if is_near(as_linear(cx, states[frame][0]), cy, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(1));
                    addch('.' as u32);
                } else if is_near(as_linear(cx, states[frame][1]), cy, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(2));
                    addch('.' as u32);
                } else {
                    attroff(0);
                    attroff(1);
                    attroff(2);
                    addch(' ' as u32);
                }
            }
            addch('\n' as u32);
        }
        refresh();
        sleep(Duration::from_millis(50));
    }
}

fn main() {
    initscr();
    start_color();
    init_pair(0, COLOR_GREEN, COLOR_BLACK);
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    noecho();
    curs_set(ncurses::CURSOR_VISIBILITY::CURSOR_INVISIBLE);
    let res = probe_strategy(|_| (0.0, 0.0), |_| (0.25, 0.26));
    // println!("{:?}", res);
    graphics_driver(res);
    refresh();
    getch();
    endwin();
}
// investigate if rotation is working 
// polish & add comments
