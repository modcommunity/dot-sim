use super::*;

/// A test variant: (kind, number). Kind 0 blends linearly, kind 1 is discrete.
#[derive(Clone, Debug, PartialEq)]
struct T(u8, f64);

impl Other for T {
    fn same_kind(&self, other: &Self) -> bool {
        self.0 == other.0
    }
    fn blend(&self, to: &Self, alpha: f64) -> Option<Self> {
        (self.0 == 0).then_some(T(0, self.1 + (to.1 - self.1) * alpha))
    }
}

type I = Interp<T, ()>;

const POS: u32 = 1;
const YAW: u32 = 2;
const FLAG: u32 = 3;
const AIM: u32 = 4;

const CFG: Config = Config {
    max_ticks: 6.0,
    ease_max: 4.0,
    ease_sec: 0.1,
};

fn pos(x: f32) -> Values<T> {
    vec![(POS, Value::Vec3([x, 0.0, -x]))]
}

fn get(values: &Values<T>, key: u32) -> Value<T> {
    find(values, key).cloned().expect("key")
}

fn ticks(interp: &I, id: i64) -> Vec<i64> {
    let track = interp.track(id).expect("track");
    track.samples.iter().map(|s| s.borrow().tick).collect()
}

#[test]
fn push_orders_merges_and_caps() {
    let mut interp = I::default();
    interp.push(1, 10, pos(1.0));
    interp.push(1, 14, pos(3.0));
    interp.push(1, 12, pos(2.0));
    assert_eq!(ticks(&interp, 1), vec![10, 12, 14]);

    interp.push(
        1,
        12,
        vec![
            (YAW, Value::Float(0.5)),
            (POS, Value::Vec3([9.0, 9.0, 9.0])),
        ],
    );
    let track = interp.track(1).unwrap();
    let merged = track.samples[1].borrow();
    assert_eq!(merged.values[0], (POS, Value::Vec3([9.0, 9.0, 9.0])));
    assert_eq!(merged.values[1], (YAW, Value::Float(0.5)));
    drop(merged);

    for t in 20..60 {
        interp.push(1, t, pos(t as f32));
    }
    assert_eq!(interp.track(1).unwrap().len(), TRACK_CAPACITY);
    assert_eq!(ticks(&interp, 1)[0], 28);
}

#[test]
fn sample_blends_holds_and_counts() {
    let mut interp = I::default();
    assert!(interp.sample(1, 5.0, &CFG).is_empty());
    interp.push(1, 10, pos(1.0));
    assert_eq!(interp.sample(1, 99.0, &CFG), pos(1.0));
    assert_eq!(interp.counts, Counts::default());

    interp.push(
        1,
        14,
        vec![
            (POS, Value::Vec3([3.0, 0.0, -3.0])),
            (YAW, Value::Float(2.0)),
            (FLAG, Value::Other(T(1, 7.0))),
        ],
    );
    interp.push(
        1,
        12,
        vec![
            (POS, Value::Vec3([2.0, 0.0, -2.0])),
            (FLAG, Value::Other(T(1, 5.0))),
            (AIM, Value::Other(T(0, 1.0))),
        ],
    );
    // Between 12 and 14 at a quarter: newer's keys, then the older-only one.
    let out = interp.sample(1, 12.5, &CFG);
    assert_eq!(out[0], (POS, Value::Vec3([2.25, 0.0, -2.25])));
    assert_eq!(out[1], (YAW, Value::Float(2.0)));
    assert_eq!(out[2], (FLAG, Value::Other(T(1, 5.0))));
    assert_eq!(out[3], (AIM, Value::Other(T(0, 1.0))));
    // Past the midpoint the discrete value switches.
    assert_eq!(
        get(&interp.sample(1, 13.0, &CFG), FLAG),
        Value::Other(T(1, 7.0))
    );
    // Older than everything: the oldest.
    assert_eq!(interp.sample(1, 3.0, &CFG), pos(1.0));
    assert_eq!(
        interp.counts,
        Counts {
            samples: 3,
            stalls: 0,
            extrapolations: 0
        }
    );
}

#[test]
fn extrapolates_positions_up_to_the_cap() {
    let mut interp = I::default();
    interp.push(
        1,
        10,
        vec![
            (POS, Value::Vec3([0.0, 0.0, 0.0])),
            (YAW, Value::Float(1.0)),
        ],
    );
    interp.push(
        1,
        12,
        vec![
            (POS, Value::Vec3([2.0, 4.0, 0.0])),
            (YAW, Value::Float(3.0)),
        ],
    );

    // On the newest: a stall, held.
    assert_eq!(
        get(&interp.sample(1, 12.0, &CFG), POS),
        Value::Vec3([2.0, 4.0, 0.0])
    );
    // Ahead: positions continue, a float holds.
    let out = interp.sample(1, 15.0, &CFG);
    assert_eq!(out[0], (POS, Value::Vec3([5.0, 10.0, 0.0])));
    assert_eq!(out[1], (YAW, Value::Float(3.0)));
    // Past the cap: held.
    assert_eq!(
        get(&interp.sample(1, 18.5, &CFG), POS),
        Value::Vec3([2.0, 4.0, 0.0])
    );
    assert_eq!(
        interp.counts,
        Counts {
            samples: 3,
            stalls: 3,
            extrapolations: 1
        }
    );
}

#[test]
fn resume_eases_the_jump_and_skips_a_teleport() {
    let mut interp = I::default();
    interp.push(1, 10, pos(0.0));
    interp.push(1, 12, pos(2.0));
    // Extrapolating at 13: x = 3.
    let out = interp.draw(1, 13.0, 0.016, &CFG).unwrap();
    assert_eq!(get(&out, POS), Value::Vec3([3.0, 0.0, -3.0]));

    // The stream resumes lower than the guess: what the guess would draw now (3.5) less the
    // drawn value is carried and decays.
    interp.push(1, 14, pos(2.5));
    let out = interp.draw(1, 13.5, 0.016, &CFG).unwrap();
    let drawn: f32 = 2.0 + (2.5 - 2.0) * 0.75;
    let jump = 3.5 - drawn;
    let keep = (-0.016f64 / 0.1).exp() as f32;
    let off = jump * keep;
    assert_eq!(
        get(&out, POS),
        Value::Vec3([drawn + off, 0.0, -drawn + -off])
    );

    // A teleport is not eased: a fresh stream far away.
    let mut far = I::default();
    far.push(1, 10, pos(0.0));
    far.push(1, 12, pos(2.0));
    far.draw(1, 13.0, 0.016, &CFG).unwrap();
    far.push(1, 14, pos(100.0));
    let out = far.draw(1, 13.5, 0.016, &CFG).unwrap();
    let drawn: f32 = 2.0 + (100.0 - 2.0) * 0.75;
    assert_eq!(get(&out, POS), Value::Vec3([drawn, 0.0, -drawn]));
    assert!(far.track(1).unwrap().offsets.is_empty());
}

#[test]
fn easing_is_off_until_frames_are_driven() {
    let mut interp = I::default();
    interp.push(1, 10, pos(0.0));
    interp.push(1, 12, pos(2.0));
    interp.draw(1, 13.0, 0.0, &CFG).unwrap();
    assert!(interp.track(1).unwrap().ahead_from.is_none());
}

#[test]
fn a_merge_reaches_the_samples_remembered() {
    let mut interp = I::default();
    interp.push(1, 10, pos(0.0));
    interp.push(1, 12, pos(2.0));
    interp.draw(1, 13.0, 0.016, &CFG).unwrap();
    interp.push(1, 12, pos(4.0));
    let track = interp.track(1).unwrap();
    let (newest, _) = track.ahead_from.as_ref().unwrap();
    assert_eq!(newest.borrow().values, pos(4.0));
}

#[test]
fn forget_clear_and_prune() {
    let mut interp = I::default();
    for id in 1..=4 {
        interp.push(id, 1, pos(0.0));
    }
    interp.forget(2);
    interp.prune(&[1, 3, 9]);
    assert_eq!(interp.track_count(), 2);
    assert!(interp.track(4).is_none());
    interp.clear();
    assert_eq!(interp.track_count(), 0);
}
