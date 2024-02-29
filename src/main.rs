// =======================================================================================================
// Importes de librerías
// =======================================================================================================

use core::panic;
use std::{
    f32::{
        consts::{PI, SQRT_2},
        NAN,
    },
    ops::{Add, Sub},
    sync::mpsc::{channel, Receiver},
    thread::{self, sleep},
    time::Duration,
};

use ncurses::{
    addch, attroff, attrset, clear, curs_set, endwin, getch, getmaxx, getmaxy, init_pair, initscr,
    noecho, refresh, start_color, stdscr, COLOR_BLACK, COLOR_BLUE, COLOR_GREEN, COLOR_PAIR,
    COLOR_RED,
};

// =======================================================================================================
// Constantes
// =======================================================================================================

const X_INIT_POS: f32 = 10.0;                         // Coordenada x del estado inicial de los
                                                      // robots

const SUMO_SIZE: f32 = 2.5;                           // Lado de un robot (son cuadrados)

const DRAWING_BOUNDRY_SIZE: f32 = 0.5;                // Sensibilidad del adaptador gráfico

const ORIGIN: Vec2 = Vec2 { x: 0.0, y: 0.0 };         // Punto 0 en el plano

const REQ_CASTER: SumoState = SumoState {             // Valor que nos permitirá hacer gimnasia de
                                                      // tipos
    center: ORIGIN,                                   //
    dir: 0.0,                                         //
    corners: [ORIGIN, ORIGIN, ORIGIN, ORIGIN],        //
};                                                    

const TATAMI_SIZE: f32 = 20.0;                        // Radio del "tatami" (circulo de lucha)

const PUSH_FRICTION: f32 = 1.0;                       // Coefficiente de fricción (no en unidades
                                                      // del sistema internacional)

const M: f32 = SUMO_SIZE * 4.0 * (SQRT_2 - 1.0) / PI; // aun más gimnasia mental


// =======================================================================================================
// Código
// =======================================================================================================

// Aclaraciones previas:
// Para hacernos la vida más fácil, declaramos tipos (estructuras y
// enumeraciones), los cuales nos ayudan a almacenar información útil
// durante el programa de manera estructurada


#[derive(Clone, Copy, Debug, PartialEq)] // | De esta estructura "derivamos"
//       ^^^^^^^^^^^  ^^^^^  ^^^^^^^^^      | ciertas propiedades que hubieramos
//       nos permite  ver el  nos permite   | tenido que implementar a mano
//       copiar la    valor   igualar       |
//       memoria de   de los  datos         |
//       la estruc-   datos                 |
//       tura                               |
//                                          |
// -------------------|---------------------|
pub struct Vec2 { //  | Esto es una estructura con dos "campos" 
                  //  |
    x: f32,       //  | un campo "x" que almacena un "f32"
                  //  | (número con decimales)
                  //  | 
    y: f32,       //  | y otro campo "y" que almacena otro 
                  //  | "f32"
}                 //  |
// -------------------| Utilizaremos esta estructura como un vector de 2 dimensiones

// -------------|
enum Round { // | Esto es una enumeración con 3 "variantes"
             // | este tipo datos podrá tener una de estas 3
             // | formas
    Round1,  // | 
    Round2,  // |
    Round3   // |
}            // |
//--------------| Utilizaremos esta enumeración para ver en que ronda estamos

// --------------------------------------------|
impl From<u8> for Round {                   // | Esto es una propiedad no derivable
                                            // |
   fn from(value: u8) -> Self { // ----------- | Convierte un tipo "u8" (unsigned 8 bits,
                                            // | entero sin signo) en una de las variantes de ronda
                                            // |
       match value {                        // |-| Esto es el valor que evaluaría llamar `from`
           1 => Round::Round1,              // | | sobre otro: `1` |-> `Round1`, `2` |-> `Round2`,
           2 => Round::Round2,              // | | `3` |-> `Round3`, `_` |-> Error
           3 => Round::Round3,              // | | "_" es un valor que es "igual" a cualquier otro,
           _ => panic!("bad round number"), // | | esto es útil ya que Rust te fuerza a comprobar
       }                                    // | | todas las ramas, "_" te permite definir un caso
                                            // |-| base
                                            // |
                                            // | Código escrito en Rust se compone de "expresiones"
                                            // | (código que evalua a un valor, `x + 1` es una
                                            // | expresión), las cuales "anulamos" (`x + 1;` es una 
                                            // | expresión anulada) para que los "bloques" 
                                            // | (código entre llaves, {`x + 1`} es un bloque) 
                                            // | solo tenga un valor
   }                                        // |
}                                           // |
// --------------------------------------------|

// ---------------------------------------------------------------|
impl Vec2 {                                                    // | Estos son propiedades propias de
                                                               // | tipo, en este caso de `Vec2`
    fn round(self) -> Vec2 { // redondea los valores internos  // |
        Vec2 {                                                 // |
            x: self.x.round(),                                 // |
            y: self.y.round(),                                 // |
        }                                                      // |
    }                                                          // |
                                                               // |
    fn origin_dir(self) -> f32 { // calcula la dirección       // |
        f32::atan(self.x / self.y)                             // |
    }                                                          // |
                                                               // |
    fn dist(self, sstate: Self) -> f32 { // calcula la         // |
                                         // distancia          // |
                                         // al origne          // |
        let vstate = self - sstate;                            // |
        f32::sqrt(vstate.x * vstate.x + vstate.y * vstate.y)   // |
    }                                                          // |
}                                                              // |
// ---------------------------------------------------------------|

type Corners = [Vec2; 4]; // ¡CUIDADO! esto es solo un alias,
                          // otro nombre para `[Vec2; 4]`,
                          // una lista de 4 `Vec2`

type Strategy = fn(f32) -> SumoReq;
// significa: f(un_valor: f32) -produce-> otro_valor: SumoReq (lo definiermos más adelante) 

impl Add<Vec2> for Vec2 { // la propiedad `Add` nos permite usar el operador `+`
//       ---       ---
//        -----------
//  implementamos para la suma 
//  Vec2 con Vec2 ...
//
//  y con resultado Vec2
//         _________
//       ______   ____
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Self::Output {
        Vec2 { // los valores de estructura se construyen tal que así
               // Estructura { Campo1: ..., Campo2: ..., ..., CampoN: ... }
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        } // nótese que no anulamos este valor, por lo que es a lo que esto evalua
    }
}

impl Sub<Vec2> for Vec2 { // similarmente, `Sub` nos permite usar el operador `-`
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SumoState {   // esta es la estructura principal del programa,
                         // representa el estado de un robot
    center: Vec2,     // guardamos el valor del centro del robot,
    dir: f32,         // la dirección del robot,
    corners: Corners, // y las esquinas (falicitará el adaptador de graficos)
}

#[derive(Clone, Copy)]
pub struct SumoReq { // las estrategias nos darán un valor de este tipo
    motor_l: f32, // velocidad del motor izquierdo
    motor_r: f32, // velocidad del motor derecho
}

impl Add<Vec2> for SumoState {
    // la suma representa mover todos los puntos del robot por el vector
    type Output = SumoState;
    fn add(self, rhs: Vec2) -> Self::Output {
        SumoState {
            center: rhs + self.center,
            dir: self.dir,
            corners: self.corners.map(|corner| corner + rhs),
            //                    ^^^
            // por cada valor de la lista, sumamos el valor con `rhs`
        }
    }
}

impl SumoState {
    // ???
    // calcula el "radio" del cuadrado, 
    // teniendo en cuenta la dirección relativa del robot
    // ???
    fn radius_towards(self, theta: f32) -> f32 {
        let rel_dir = (theta - self.dir) % (PI / 2.0); // definimos una variable temporal
                                                       // el acceso de cualquier modo a
                                                       // esta variable fuera de su "mira"
                                                       // resultará en error
        if rel_dir <= PI / 4.0 {
            M * rel_dir + SUMO_SIZE // asumo que esta relación es linear
        } else {
            M * (PI / 2.0 - rel_dir) + SUMO_SIZE
        }
    }
    // `rel_dir` es inaccesible aquí
}

impl SumoReq {
    fn vel(self) -> f32 {
        (self.motor_l + self.motor_r) / 2.0 // implementación formula de la 
                                            // velocidad del accionamiento
                                            // differencial
    }
}

impl Add<SumoReq> for SumoState {
    type Output = SumoState;
    fn add(self, sstate: SumoReq) -> SumoState {
        let delta_teta =
            (((sstate.motor_r - sstate.motor_l) / SUMO_SIZE) + self.dir) % (2.0 * PI);
            // usamos la formula del ángulo resultante en el accionamiento differencial ...
        let vel = sstate.vel();
            // además de calcular la velocidad ...

        let dv = Vec2 {
            x: delta_teta.cos() * vel,
            y: delta_teta.sin() * vel,
        };  // para calcular el vector de "fuerza" (desconozco si este es el termino)

        SumoState {
            center: self.center + dv,
            dir: delta_teta,
            corners: self.corners.map(|corner| corner + dv),
        }
    }
}

fn is_near<T>(x: T, y: T, bound: T) -> bool
where
    T: Add<Output = T>, // Gimnasias de tipos, no es importante esta parte
    T: Sub<Output = T>, // Basicamente esta función es compatible con cualquier
    T: PartialOrd,      // tipo de datos si cumple estas propiedades
    T: Copy,            //
{
    x < y + bound && x > y - bound // este es el valor devuelto
}

// Este es el motor de este código
// Es el encargado de calcular los estados de los robots, ver si existen colisiones,
// aplicar los vectores de fuerza resultantes, y calcular la respuesta de los robots
// dado la respuesta de los sensores infrarojos que tambien calcula esta función
//
pub fn probe_strategy(strat1: Strategy, strat2: Strategy) -> Receiver<[SumoState; 2]> {
    // Para mantener mi sanidad, he roto el problema en trozos manejables y modulares:
    
    // Esta función dado un número de ronda devuelve el estado inicial en el que deberían de estar
    // los robots
    // Su longitud se debe al estilizado de código
    fn round_start(round: Round) -> [SumoState; 2] {[
        SumoState {
            center: Vec2 {
                x: X_INIT_POS,
                y: 0.0,
            },
            dir: match round {
                Round::Round1 => PI,
                Round::Round2 => PI/2.0,
                Round::Round3 => 0.0,
            },
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
            dir: match round {
                Round::Round1 => 0.0,
                Round::Round2 => 3.0*PI/2.0,
                Round::Round3 => PI,
            },

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
    ]}

    // Siguiendo la analogía del motor, esta función sería el bloque motor del motor, es la
    // encargada de toda la lógica
    fn update(
        [sysl, sysr]: [SumoState; 2],
//       -----------
//  ya que la lista tiene exactamente dos entradas, Rust nos deja descomponer la lista de esta
//  manera
//       __________
        [reql, reqr]: [SumoReq; 2],
    ) -> Option<[SumoState; 2]> {
        let mut sy_s = [sysl + reql, sysr + reqr]; // actualizamos la lista de dos entradas con los
                                                   // estados previos para sumarle las respuesta de
                                                   // sus correspondientes motores
        let theta = (sy_s[0].center - sy_s[1].center).origin_dir(); // calculamos el ángulo entre
                                                                    // el centro de los dos robots

        if sysl.center.dist(sysr.center) // | por lo que la comparamos con la distancia entre real
                                         // | entre estos dos
                                         // |___
            < sy_s[0].radius_towards(theta) // ----| La suma de los "radios" en el ángulo calculado
                + sy_s[1].radius_towards(theta) // | previamente resulta en el menor valor en el
                                                // | que los dos robots no se están tocando ...
        {
            let vatt = reqr.vel() - reql.vel();
            let ap = if vatt > 0.0 {
                |x, y| x + y
            } else {
                |x, y| x + (ORIGIN - y)
            };
            let (gyatt, xatt) = (REQ_CASTER + reqr).dir.sin_cos();
            let vec_push = Vec2 {
                x: xatt * vatt / PUSH_FRICTION,
                y: gyatt * vatt / PUSH_FRICTION,
            };
            sy_s = [ap(sysl, vec_push), ap(sysr, ORIGIN - vec_push)];
        }; // Sinceramente, no sé que hice aquí, tendría que redescrubrir en lo que he pensado para
           // explicarlo
           //
           // Basicamente aplica el vector fuerza si existe colisión

        if sysl.center.dist(ORIGIN) < TATAMI_SIZE && sysr.center.dist(ORIGIN) < TATAMI_SIZE {
            // Comprobamos si los dos robots siguen en el tatami

            Some(sy_s) // Si esto es el caso, la partida sigue, tenemos "algún" estado
        //  ---- de ahí este constructor
        } else {
            None // Si no, la partida se ha acabado, no tenemos estado
        //  ---- de ahí este constructor
        }
        // Nótese que no podríamos simplemente devolver `sy_s` y un valor nulo, Rust no tiene
        // valores nulos ya que son un peligro para la seguridad y las mentes de los programadores
    }

    // Calcula la señal del sensor infrarojos (realmente es ultrasonidos pero queda más feo)
    //
    // Este valor no es exactamente el valor del sensor, si no la distancia del centro entre los
    // dos robots (si llegan a verse, si no, devuelve 0)
    //
    // !!!
    // Si conoce aluna formula relativamente sencilla de calcular este valor exacto, por favor
    // comentemelo
    // !!!
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

    // Finalmente, esta es la lógica principal
    //
    // No importa mucho los detalles de la implementación
    // Básicamente crea un canal de comunicación entre "hilos" (tareas que su ordenador ejecuta de
    // manera simultánea) para que los valores calculados puedan ser representados por el adaptador
    // gráfico de manera inmediata
    let (s_tx, states) = channel();
    thread::spawn(move || {
    for round in 1u8..=3u8 {
        let mut sym_state = round_start(Round::from(round));
        loop {
            let dist = sym_state[0].center.dist(sym_state[1].center);
            let ir_reads: [SumoReq; 2] = [
                strat1(calc_ir(sym_state[0], sym_state[1], dist)),
                strat2(calc_ir(sym_state[1], sym_state[0], dist)),
            ];
            match update(sym_state, ir_reads) {
                Some(symst) => sym_state = symst,
                None => break,
            };
            s_tx.send(sym_state).unwrap();
        }
    }});
    states
}

// Este es el adaptador gráfico
// No importa mucho los detalles de su implementación y me avergüenza haber escrito código tan feo
pub fn graphics_driver(states: Receiver<[SumoState; 2]>) {
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
    if states.try_recv().is_err() {
        sleep(Duration::from_millis(50))
    }
    while let Ok(frame) = states.try_recv() {
        clear();
        for y in 0..maxy {
            for x in 0..maxx - 1 {
                let cx = (x / 2 - maxx / 4) as f32;
                let cy = (y - maxy / 2) as f32;
                let d = f32::sqrt((cx * cx) as f32 + (cy * cy) as f32);
                if frame[0].center.x.round() == cx && frame[0].center.y.round() == cy {
                    attrset(COLOR_PAIR(1));
                    addch('=' as u32);
                } else if frame[1].center.x.round() == cx && frame[1].center.y.round() == cy {
                    attrset(COLOR_PAIR(2));
                    addch('=' as u32);
                } else if frame[0]
                    .corners
                    .map(|corner| corner.round())
                    .contains(&Vec2 { x: cx, y: cy })
                {
                    attrset(COLOR_PAIR(1));
                    addch('o' as u32);
                } else if frame[1]
                    .corners
                    .map(|corner| corner.round())
                    .contains(&Vec2 { x: cx, y: cy })
                {
                    attrset(COLOR_PAIR(2));
                    addch('o' as u32);
                } else if is_near(d, TATAMI_SIZE as f32, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(0));
                    addch('#' as u32);
                } else if is_near(as_linear(cx, frame[0]), cy, DRAWING_BOUNDRY_SIZE) {
                    attrset(COLOR_PAIR(1));
                    addch('.' as u32);
                } else if is_near(as_linear(cx, frame[1]), cy, DRAWING_BOUNDRY_SIZE) {
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

// Este es nuestro punto de entrada al programa, por donde empezará a correr el ordenador el código
fn main() {
    initscr();                                                 // Funciones inicializadores de la
    start_color();                                             // librería gráfica `ncurses`
    init_pair(0, COLOR_GREEN, COLOR_BLACK);                    //
    init_pair(1, COLOR_BLUE, COLOR_BLACK);                     //
    init_pair(2, COLOR_RED, COLOR_BLACK);                      //
    noecho();                                                  //
    curs_set(ncurses::CURSOR_VISIBILITY::CURSOR_INVISIBLE);    //

    let res = probe_strategy( // `res` será el transmisor entre hilos
        |_| SumoReq {
            motor_l: 0.0,
            motor_r: 0.0
        }, 
        |_| SumoReq {
            motor_r: 0.25,
            motor_l: 0.25
        }
    );
    // println!("{:?}", res); // comentario útil
    thread::spawn(move || graphics_driver(res)).join().unwrap(); // le damos `res` al adaptador
                                                                 // gráfico, que traducirá el
                                                                 // estado de la batalla en
                                                                 // gráficos `ncurses`
    getch();  // Esperamos a que el usuario responda del shock de semejante batalla
    endwin(); // Terminamos la ventana del simulador
}
