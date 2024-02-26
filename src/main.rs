use std::{f32::consts::PI, ops::{Add, Sub}, sync::{Arc, RwLock, Barrier, Mutex}, thread::spawn};

const STEP_SIZE    : f32 = 0.1;
const X_INIT_POS   : f32 = 10.0;
const SUMO_SIZE    : f32 = 2.5;
const BOUND_SIZE   : f32 = 0.5;
const ORIGIN       : SumoState = SumoState { x: 0.0, y: 0.0, dir: 0.0};
const TATAMI_SIZE  : f32 = 20.0;
const PUSH_FRICTION: f32 = 10.0;

#[derive(Clone, Copy, Debug)]
struct SumoState {
    x: f32,
    y: f32,
    dir: f32
}
impl SumoState {
    fn dist(self, sstate: Self) -> f32 {
        let vstate = self - sstate;
        f32::sqrt(vstate.x*vstate.x + vstate.y*vstate.y)
    }

}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Robocillo {
    Paco,
    Curro
}
impl Robocillo {
    fn select<T>(self, x: T, y: T) -> T {
        if self == Robocillo::Curro { x } else { y }
    }
}

#[derive(Clone, Copy)]
struct SumoReq {
    sumo: Robocillo,
    motor_l: f32,
    motor_r: f32
}
impl SumoReq {
    fn vel(self) -> f32 {
        (self.motor_l + self.motor_r)/2.0
    }
}

impl Into<SumoState> for SumoReq {
    fn into(self) -> SumoState {
        let theta_p = STEP_SIZE*(self.motor_r - self.motor_l)/SUMO_SIZE;
        let vel = self.vel();
        SumoState { x: theta_p.cos()*vel, y: theta_p.sin()*vel, dir: theta_p }
    }
}

impl Add<SumoState> for SumoState {
    type Output = SumoState;
    fn add(self, sstate: SumoState) -> SumoState {
        SumoState { x: self.x + sstate.x, y: self.y + sstate.y, dir: self.dir + sstate.dir }
    }
}
impl Sub<SumoState> for SumoState {
    type Output = SumoState;
    fn sub(self, sstate: SumoState) -> SumoState {
        SumoState { x: self.x - sstate.x, y: self.y - sstate.y, dir: self.dir + sstate.dir }
    }
}

impl Add<SumoReq> for (SumoState, SumoState) {
    type Output = (SumoState, SumoState);
    fn add(self, sreq: SumoReq) -> (SumoState, SumoState) {
        let sstate_p = sreq.into();
        if sreq.sumo == Robocillo::Curro { (self.0, self.1 + sstate_p) } else { (self.0 + sstate_p, self.1) }
    }
}

type Strategy = fn(f32) -> (f32, f32);
pub fn create_strat_prober(strat1: Strategy, strat2: Strategy) -> std::thread::JoinHandle<()> {
    let mut sym_state = Ok((SumoState { x: X_INIT_POS, y: 0.0, dir: 0.0 }, SumoState { x: -X_INIT_POS, y: 0.0, dir: PI }));
    let sym_lock = Arc::new(RwLock::new(Ok((0.0, 0.0))));
    let (s1_lock, s2_lock) = (sym_lock.clone(), sym_lock.clone());

    let s1_moves = Arc::new(Mutex::new((to_req(Robocillo::Paco, (0.0, 0.0)), to_req(Robocillo::Paco, (0.0, 0.0)))));
    let s2_moves = s1_moves.clone();
    let ss_reads = s1_moves.clone();

    fn to_req(sumo: Robocillo, (motor_l, motor_r): (f32, f32)) -> SumoReq
        { SumoReq { sumo, motor_l, motor_r } }

    let barrier1 = Arc::new(Barrier::new(2));
    let barrier2 = barrier1.clone();

    spawn(move || loop {
        let ir_read = s1_lock.read().unwrap();
        let s1_tx = s1_moves.lock().unwrap();
        if ir_read.is_err() { break; }
        s1_tx.0 = to_req(Robocillo::Paco, dbg!(strat1(ir_read.unwrap().0)));
        barrier1.wait();
    });

    spawn(move || loop {
        let ir_read = s2_lock.read().unwrap();
        let s2_tx = s2_moves.lock().unwrap();
        if ir_read.is_err() { break; }
        s2_tx.1 = to_req(Robocillo::Paco, dbg!(strat2(ir_read.unwrap().1)));
        barrier2.wait();
    });

    spawn(move || loop {
        let ss_rx = ss_reads.lock().unwrap();
        sym_state = ss_rx.try_iter()
                       .try_fold(sym_state.unwrap(), |mut sy_s, req| {
                           sy_s = sy_s + req;
                           dbg!(sy_s);
                           if sy_s.0.dist(sy_s.1) < 2.0*SUMO_SIZE + BOUND_SIZE {
                               let vel = req.vel();
                               let (gyatt, xatt) = <SumoReq as Into<SumoState>>::into(req).dir.sin_cos();
                               let attacked = req.sumo.select(&mut sy_s.0, &mut sy_s.1);
                               *attacked = *attacked - SumoState { x: xatt*vel/PUSH_FRICTION, y: gyatt*vel/PUSH_FRICTION, dir: 0.0 };
                           };
                           if dbg!(sy_s.0.dist(ORIGIN)) < TATAMI_SIZE { 
                            if dbg!(sy_s.1.dist(ORIGIN)) < TATAMI_SIZE 
                               { Ok(sy_s) } else { Err(Robocillo::Paco) }
                           } else { Err(Robocillo::Curro) }
                       });
        let mut write = sym_lock.write().unwrap();

        match dbg!(sym_state) {
            Ok((s1, s2)) => *write = Ok((s1.dir, s2.dir)),
            Err(ganador) => *write = Err(ganador)
        };
    })

}

fn main() {
    create_strat_prober(|_| (0.5, 0.5), |_| (0.5, 0.5)).join().unwrap();
}
