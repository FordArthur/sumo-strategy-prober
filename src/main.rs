use std::{
    f32::consts::PI,
    ops::{Add, Sub}, thread::sleep, time::Duration,
};

use ncurses::{
    addch, attroff, attrset, clear, endwin, getch, getmaxx, getmaxy, init_pair, initscr, refresh, start_color, stdscr, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_PAIR, COLOR_RED
};

const STEP_SIZE: f32 = 0.1;
const X_INIT_POS: f32 = 10.0;
const SUMO_SIZE: f32 = 2.5;
const COLLISION_BOUNDRY_SIZE: f32 = 0.5;
const DRAWING_BOUNDRY_SIZE: f32 = 0.5;
const ORIGIN: SumoState = SumoState {
    x: 0.0,
    y: 0.0,
    dir: 0.0,
};
const TATAMI_SIZE: f32 = 20.0;
const PUSH_FRICTION: f32 = 1.0;

#[derive(Clone, Copy, Debug)]
pub struct SumoState {
    x: f32,
    y: f32,
    dir: f32,
}
impl SumoState {
    fn dist(self, sstate: Self) -> f32 {
        let vstate = self - sstate;
        f32::sqrt(vstate.x * vstate.x + vstate.y * vstate.y)
    }
    fn origin_dir(self) -> f32 {
        f32::atan(self.x / self.y)
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

impl Into<SumoState> for SumoReq {
    fn into(self) -> SumoState {
        let theta_p = STEP_SIZE * (self.motor_r - self.motor_l) / SUMO_SIZE;
        let vel = self.vel();
        SumoState {
            x: theta_p.cos() * vel,
            y: theta_p.sin() * vel,
            dir: theta_p,
        }
    }
}

impl Add<SumoState> for SumoState {
    type Output = SumoState;
    fn add(self, sstate: SumoState) -> SumoState {
        SumoState {
            x: self.x + sstate.x,
            y: self.y + sstate.y,
            dir: (self.dir + sstate.dir) % (2.0 * PI),
        }
    }
}
impl Sub<SumoState> for SumoState {
    type Output = SumoState;
    fn sub(self, sstate: SumoState) -> SumoState {
        SumoState {
            x: self.x - sstate.x,
            y: self.y - sstate.y,
            dir: self.dir + sstate.dir,
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
            x: X_INIT_POS,
            y: 0.0,
            dir: PI
        },
        SumoState {
            x: -X_INIT_POS,
            y: 0.0,
            dir: 0.0,
        },
    ];

    fn as_req((motor_l, motor_r): (f32, f32)) -> SumoReq {
        SumoReq { motor_l, motor_r }
    }

    fn update(
        [sysl, sysr]: [SumoState; 2],
        [reql, reqr]: [SumoReq; 2],
    ) -> Result<[SumoState; 2], Robocillo> {
        let mut sy_s = [sysl + reql.into(), sysr + reqr.into()];
        if sysl.dist(sysr) < 2.0 * SUMO_SIZE + COLLISION_BOUNDRY_SIZE {
            let vatt = reqr.vel() - reql.vel();
            let (gyatt, xatt) = <SumoReq as Into<SumoState>>::into(reqr).dir.sin_cos();
            let vec_push = SumoState {
                x: xatt * vatt / PUSH_FRICTION,
                y: gyatt * vatt / PUSH_FRICTION,
                dir: 0.0,
            };
            sy_s = [sysl + vec_push, sysr - vec_push];
        };
        if sysl.dist(ORIGIN) < TATAMI_SIZE {
            if sysr.dist(ORIGIN) < TATAMI_SIZE {
                Ok(sy_s)
            } else {
                Err(Robocillo::Paco)
            }
        } else {
            Err(Robocillo::Curro)
        }
    }

    fn calc_ir(s1: SumoState, s2: SumoState, dist: f32) -> f32 {
        let glob_dir = s2.origin_dir();
        if s1.dir < (glob_dir + SUMO_SIZE / 2.0) % (2.0 * PI)
            && s1.dir > (glob_dir - SUMO_SIZE / 2.0) % (2.0 * PI)
        {
            dist
        } else {
            0.0
        }
    }

    let mut states = Vec::new();
    loop {
        let dist = sym_state[0].dist(sym_state[1]);
        let ir_reads: [SumoReq; 2] = [
            as_req(strat1(calc_ir(sym_state[0], sym_state[1], dist))),
            as_req(strat2(calc_ir(sym_state[1], sym_state[1], dist))),
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

    for frame in 0..states.len() {
        clear();
        for y in 0..maxy {
            for x in 0..maxx - 1{
                let cx = x / 2 - maxx / 4;
                let cy = y - maxy / 2;
                let d = f32::sqrt((cx * cx) as f32 + (cy * cy) as f32);
                if states[frame][0].x.round() == cx as f32
                    && states[frame][0].y.round() == cy as f32
                {
                    attrset(COLOR_PAIR(1));
                    addch('=' as u32);
                } else if states[frame][1].x.round() == cx as f32
                    && states[frame][1].y.round() == cy as f32
                {
                    attrset(COLOR_PAIR(2));
                    addch('=' as u32);
                } else if is_near(d, TATAMI_SIZE as f32, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(0));
                    addch('#' as u32);
                }/*  else if is_near(
                    cx as f32 * states[frame][0].dir.tan(),
                    cy as f32,
                    DRAWING_BOUNDRY_SIZE,
                ) {
                    attrset(COLOR_PAIR(1));
                    addch('.' as u32);
                } else if is_near(
                    cx as f32 * states[frame][1].dir.tan(),
                    cy as f32,
                    DRAWING_BOUNDRY_SIZE,
                ) {
                    attrset(COLOR_PAIR(2));
                    addch('.' as u32);
                }  */else {
                    attroff(0); 
                    attroff(1); 
                    attroff(2); 
                    addch(' ' as u32);
                }
            }
            addch('\n' as u32);
        }
        refresh();
        sleep(Duration::from_millis(250));
    }
}

fn main() {
    initscr();
    start_color();
    init_pair(0, COLOR_GREEN, COLOR_BLACK);
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);

    let res = probe_strategy(|_| (-0.5, -0.5), |_| (0.0, 0.0));
    graphics_driver(res);
    getch();
    endwin();
}
