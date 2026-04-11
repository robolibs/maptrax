use crate::core::{MaptraxError, aabb_center, point_distance, segment_length};
use crate::field::{Field, Part, Ring, Swath, SwathType};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DivisionType {
    Block,
    Alternate,
    SpatialRtree,
    LengthBalanced,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct DivisionResult {
    pub headlands_per_machine: Vec<Vec<Ring>>,
    pub swaths_per_machine: Vec<Vec<Swath>>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Divy {
    part: Part,
    division_type: DivisionType,
    machine_count: usize,
    division: DivisionResult,
}

impl Divy {
    pub fn new_from_part(
        part: Part,
        division_type: DivisionType,
        machines: usize,
    ) -> Result<Self, MaptraxError> {
        Self::from_part(part, division_type, machines)
    }

    pub fn new_from_field(
        field: &Field,
        division_type: DivisionType,
        machines: usize,
    ) -> Result<Self, MaptraxError> {
        Self::from_field(field, division_type, machines)
    }

    pub fn from_field(
        field: &Field,
        division_type: DivisionType,
        machines: usize,
    ) -> Result<Self, MaptraxError> {
        if machines == 0 {
            return Err(MaptraxError::InvalidMachineCount);
        }
        let part = field.part(0)?.clone();
        Ok(Self::from_part(part, division_type, machines)?)
    }

    pub fn from_part(
        part: Part,
        division_type: DivisionType,
        machines: usize,
    ) -> Result<Self, MaptraxError> {
        if machines == 0 {
            return Err(MaptraxError::InvalidMachineCount);
        }
        Ok(Self {
            part,
            division_type,
            machine_count: machines,
            division: DivisionResult::default(),
        })
    }

    pub fn set_machine_count(&mut self, machines: usize) -> Result<(), MaptraxError> {
        if machines == 0 {
            return Err(MaptraxError::InvalidMachineCount);
        }
        self.machine_count = machines;
        Ok(())
    }

    pub fn set_division_type(&mut self, division_type: DivisionType) {
        self.division_type = division_type;
    }

    pub fn result(&self) -> &DivisionResult {
        &self.division
    }

    pub fn compute_division(&mut self) {
        let machine_count = self.machine_count;
        let mut result = DivisionResult {
            headlands_per_machine: vec![Vec::new(); machine_count],
            swaths_per_machine: vec![Vec::new(); machine_count],
        };

        let headlands: Vec<Ring> = self
            .part
            .headlands
            .iter()
            .filter(|ring| !ring.finished)
            .cloned()
            .collect();
        for (index, ring) in headlands.into_iter().enumerate() {
            result.headlands_per_machine[index % machine_count].push(ring);
        }

        let swaths: Vec<Swath> = self
            .part
            .swaths
            .iter()
            .filter(|swath| !swath.finished && swath.r#type == SwathType::Swath)
            .cloned()
            .collect();

        match self.division_type {
            DivisionType::Alternate => {
                for (index, swath) in swaths.into_iter().enumerate() {
                    result.swaths_per_machine[index % machine_count].push(swath);
                }
            }
            DivisionType::Block => {
                let base = swaths.len() / machine_count;
                let rem = swaths.len() % machine_count;
                let mut index = 0;
                for machine in 0..machine_count {
                    let count = base + usize::from(machine < rem);
                    for _ in 0..count {
                        result.swaths_per_machine[machine].push(swaths[index].clone());
                        index += 1;
                    }
                }
            }
            DivisionType::LengthBalanced => {
                let mut loads = vec![0.0; machine_count];
                let mut by_length = swaths;
                by_length.sort_by(|a, b| {
                    segment_length(b.line)
                        .partial_cmp(&segment_length(a.line))
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                for swath in by_length {
                    let machine = loads
                        .iter()
                        .enumerate()
                        .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                        .map(|(index, _)| index)
                        .unwrap_or(0);
                    loads[machine] += segment_length(swath.line);
                    result.swaths_per_machine[machine].push(swath);
                }
            }
            DivisionType::SpatialRtree => {
                let mut remaining = swaths;
                for machine in 0..machine_count {
                    let base = remaining.len() / (machine_count - machine).max(1);
                    let rem = remaining.len() % (machine_count - machine).max(1);
                    let target = base + usize::from(machine < rem);
                    if target == 0 {
                        continue;
                    }

                    let first = if machine == 0 {
                        0
                    } else {
                        remaining
                            .iter()
                            .enumerate()
                            .min_by(|(_, a), (_, b)| {
                                aabb_center(a.bounding_box)
                                    .x()
                                    .partial_cmp(&aabb_center(b.bounding_box).x())
                                    .unwrap_or(std::cmp::Ordering::Equal)
                            })
                            .map(|(index, _)| index)
                            .unwrap_or(0)
                    };

                    let mut chosen = vec![remaining.remove(first)];
                    while chosen.len() < target && !remaining.is_empty() {
                        let center = aabb_center(chosen.last().unwrap().bounding_box);
                        let next = remaining
                            .iter()
                            .enumerate()
                            .min_by(|(_, a), (_, b)| {
                                point_distance(center, aabb_center(a.bounding_box))
                                    .partial_cmp(&point_distance(
                                        center,
                                        aabb_center(b.bounding_box),
                                    ))
                                    .unwrap_or(std::cmp::Ordering::Equal)
                            })
                            .map(|(index, _)| index)
                            .unwrap_or(0);
                        chosen.push(remaining.remove(next));
                    }
                    result.swaths_per_machine[machine] = chosen;
                }
            }
        }

        self.division = result;
    }
}
