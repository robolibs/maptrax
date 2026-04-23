use std::collections::HashSet;

use geo::Point;
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::{
    EdgeType, Graph, VertexId, add_edge_with_weight, add_vertex_with_property, num_edges,
    num_vertices,
};

use crate::core::{aabb_from_points, point_distance, segment_end, segment_length, segment_start};
use crate::field::{Swath, SwathType, canonical_swath_order, dominant_swath_tangent};

#[derive(Debug, Clone, PartialEq)]
pub struct ABLine {
    pub a: Point,
    pub b: Point,
    pub uuid: String,
    pub line_id: usize,
}

impl ABLine {
    pub fn new(a: Point, b: Point, uuid: impl Into<String>, line_id: usize) -> Self {
        Self {
            a,
            b,
            uuid: uuid.into(),
            line_id,
        }
    }

    pub fn length(&self) -> f64 {
        point_distance(self.a, self.b)
    }

    pub fn equal(&self, swath: &Swath) -> bool {
        self.uuid == swath.uuid
    }
}

#[derive(Debug, Clone)]
pub struct Nety {
    ab_lines: Vec<ABLine>,
    swaths: Vec<Swath>,
    graph: Graph<Point, ()>,
    vertices: Vec<(VertexId<Point>, VertexId<Point>)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoutingStrategy {
    GreedyNearest,
    Snake,
    Spiral,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutingOptions {
    pub strategy: RoutingStrategy,
    pub local_improvement_passes: usize,
}

impl Default for RoutingOptions {
    fn default() -> Self {
        Self {
            strategy: RoutingStrategy::GreedyNearest,
            local_improvement_passes: 0,
        }
    }
}

impl Nety {
    pub fn new(swaths: &[Swath]) -> Self {
        let swaths: Vec<Swath> = swaths
            .iter()
            .filter(|swath| swath.r#type == SwathType::Swath)
            .cloned()
            .collect();

        let ab_lines = swaths
            .iter()
            .enumerate()
            .map(|(index, swath)| {
                ABLine::new(
                    segment_start(swath.line),
                    segment_end(swath.line),
                    swath.uuid.clone(),
                    index,
                )
            })
            .collect::<Vec<_>>();

        let mut graph = Graph::new();
        let mut vertices = Vec::with_capacity(swaths.len());
        for swath in &swaths {
            let va = add_vertex_with_property(segment_start(swath.line), &mut graph);
            let vb = add_vertex_with_property(segment_end(swath.line), &mut graph);
            add_edge_with_weight(va, vb, segment_length(swath.line), &mut graph);
            vertices.push((va, vb));
        }

        for i in 0..vertices.len() {
            for j in (i + 1)..vertices.len() {
                let endpoints_i = [vertices[i].0, vertices[i].1];
                let endpoints_j = [vertices[j].0, vertices[j].1];
                for from in endpoints_i {
                    for to in endpoints_j {
                        let p_from = graph[from];
                        let p_to = graph[to];
                        graph.add_edge(
                            from,
                            to,
                            point_distance(p_from, p_to),
                            EdgeType::Undirected,
                            (),
                        );
                    }
                }
            }
        }

        Self {
            ab_lines,
            swaths,
            graph,
            vertices,
        }
    }

    pub fn graph(&self) -> &Graph<Point, ()> {
        &self.graph
    }

    pub fn ab_lines(&self) -> &[ABLine] {
        &self.ab_lines
    }

    pub fn get_ab_lines(&self) -> &[ABLine] {
        self.ab_lines()
    }

    pub fn swaths(&self) -> &[Swath] {
        &self.swaths
    }

    pub fn get_swaths(&self) -> &[Swath] {
        self.swaths()
    }

    pub fn num_vertices(&self) -> usize {
        num_vertices(&self.graph)
    }

    pub fn num_edges(&self) -> usize {
        num_edges(&self.graph)
    }

    pub fn field_traversal(&mut self, start_point: Option<Point>) {
        self.field_traversal_with_options(start_point, RoutingOptions::default());
    }

    pub fn field_traversal_with_options(
        &mut self,
        start_point: Option<Point>,
        options: RoutingOptions,
    ) {
        if self.swaths.is_empty() {
            return;
        }

        let start_point = start_point.unwrap_or(segment_start(self.swaths[0].line));
        let traversal = self.plan_traversal(start_point, options);
        self.swaths = build_connection_augmented_order(traversal);
    }

    pub fn shortest_path(&mut self, start: Option<Point>, goal: Option<Point>) {
        if self.swaths.is_empty() {
            return;
        }
        let start = start.unwrap_or(segment_start(self.swaths[0].line));
        let goal = goal.unwrap_or(segment_end(self.swaths[0].line));
        let (start_line, start_vertex) = self.closest_endpoint(start);
        let (goal_line, goal_vertex) = self.closest_endpoint(goal);

        let result = dijkstra(&self.graph, start_vertex, goal_vertex);
        if !result.found {
            return;
        }

        let mut seen = HashSet::new();
        let mut ordered = Vec::new();
        for window in result.path.windows(2) {
            let from = window[0];
            let to = window[1];
            if let Some((line_index, from_is_a, to_is_b)) = self.line_transition(from, to) {
                if seen.insert(line_index) {
                    let mut swath = self.swaths[line_index].clone();
                    if !(from_is_a && to_is_b) {
                        swath.swap_direction();
                    }
                    ordered.push(swath);
                }
            }
        }

        if ordered.is_empty() {
            let mut start_swath = self.swaths[start_line].clone();
            if self.vertices[start_line].1 == start_vertex {
                start_swath.swap_direction();
            }
            ordered.push(start_swath);
        }

        if !ordered
            .iter()
            .any(|swath| swath.uuid == self.swaths[goal_line].uuid)
        {
            let mut goal_swath = self.swaths[goal_line].clone();
            if self.vertices[goal_line].1 == goal_vertex {
                goal_swath.swap_direction();
            }
            ordered.push(goal_swath);
        }

        self.swaths = ordered;
    }

    pub fn calculate_path_distance(&self, path: &[VertexId<Point>]) -> f64 {
        path.windows(2)
            .map(|pair| point_distance(self.graph[pair[0]], self.graph[pair[1]]))
            .sum()
    }

    fn closest_endpoint(&self, point: Point) -> (usize, VertexId<Point>) {
        self.vertices
            .iter()
            .enumerate()
            .flat_map(|(line_index, (a, b))| [(*a, line_index), (*b, line_index)])
            .min_by(|(va, _), (vb, _)| {
                point_distance(point, self.graph[*va])
                    .partial_cmp(&point_distance(point, self.graph[*vb]))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(vertex, line_index)| (line_index, vertex))
            .unwrap()
    }

    fn line_transition(
        &self,
        from: VertexId<Point>,
        to: VertexId<Point>,
    ) -> Option<(usize, bool, bool)> {
        self.vertices
            .iter()
            .enumerate()
            .find_map(|(line_index, (a, b))| {
                if *a == from && *b == to {
                    Some((line_index, true, true))
                } else if *b == from && *a == to {
                    Some((line_index, false, false))
                } else {
                    None
                }
            })
    }

    fn plan_traversal(&self, start_point: Point, options: RoutingOptions) -> Vec<Swath> {
        let mut traversal = match options.strategy {
            RoutingStrategy::GreedyNearest => self.greedy_nearest_order(start_point),
            RoutingStrategy::Snake => self.pattern_order(start_point, RoutingStrategy::Snake),
            RoutingStrategy::Spiral => self.pattern_order(start_point, RoutingStrategy::Spiral),
        };

        if options.local_improvement_passes > 0 && traversal.len() >= 3 {
            traversal =
                improve_adjacent_swaps(traversal, start_point, options.local_improvement_passes);
        }

        traversal
    }

    fn greedy_nearest_order(&self, start_point: Point) -> Vec<Swath> {
        let mut remaining: HashSet<usize> = (0..self.swaths.len()).collect();
        let mut traversal = Vec::with_capacity(self.swaths.len());
        let mut current_point = start_point;

        while !remaining.is_empty() {
            let (index, start_from_head) = remaining
                .iter()
                .copied()
                .map(|index| {
                    let swath = &self.swaths[index];
                    let head_dist = point_distance(current_point, swath.head());
                    let tail_dist = point_distance(current_point, swath.tail());
                    if head_dist <= tail_dist {
                        (index, true, head_dist)
                    } else {
                        (index, false, tail_dist)
                    }
                })
                .min_by(|a, b| {
                    a.2.partial_cmp(&b.2)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then_with(|| a.0.cmp(&b.0))
                })
                .map(|(index, start_from_head, _)| (index, start_from_head))
                .unwrap();

            remaining.remove(&index);
            let mut swath = self.swaths[index].clone();
            if !start_from_head {
                swath.swap_direction();
            }
            current_point = swath.tail();
            traversal.push(swath);
        }

        traversal
    }

    fn pattern_order(&self, start_point: Point, strategy: RoutingStrategy) -> Vec<Swath> {
        let tangent = dominant_swath_tangent(&self.swaths);
        let canonical = canonical_swath_order(&self.swaths);

        let ordered_indices = match strategy {
            RoutingStrategy::Snake => snake_indices(canonical),
            RoutingStrategy::Spiral => spiral_indices(canonical, start_point, &self.swaths),
            RoutingStrategy::GreedyNearest => unreachable!(),
        };

        let mut traversal = ordered_indices
            .into_iter()
            .enumerate()
            .map(|(order_index, swath_index)| {
                let mut swath = self.swaths[swath_index].clone();
                let positive = order_index % 2 == 0;
                if direction_dot(&swath, tangent) < 0.0 && positive {
                    swath.swap_direction();
                } else if direction_dot(&swath, tangent) > 0.0 && !positive {
                    swath.swap_direction();
                }
                swath
            })
            .collect::<Vec<_>>();

        maybe_reverse_for_start(&mut traversal, start_point);
        traversal
    }
}

fn build_connection_augmented_order(traversal: Vec<Swath>) -> Vec<Swath> {
    let mut ordered = Vec::with_capacity(traversal.len().saturating_mul(2));
    for (index, swath) in traversal.iter().cloned().enumerate() {
        ordered.push(swath.clone());
        if let Some(next) = traversal.get(index + 1) {
            let connection = Swath {
                line: geo::Line::new(swath.tail().0, next.head().0),
                uuid: format!("connection_{}_{}", swath.uuid, next.uuid),
                r#type: SwathType::Connection,
                finished: false,
                bounding_box: aabb_from_points(&[swath.tail(), next.head()]),
                id: -1,
                width: 0.0,
                points: vec![swath.tail(), next.head()],
                point_reverse: Vec::new(),
            };
            ordered.push(connection);
        }
    }
    ordered
}

fn direction_dot(swath: &Swath, tangent: (f64, f64)) -> f64 {
    (swath.tail().x() - swath.head().x()) * tangent.0
        + (swath.tail().y() - swath.head().y()) * tangent.1
}

fn snake_indices(indices: Vec<usize>) -> Vec<usize> {
    indices
}

fn spiral_indices(indices: Vec<usize>, start_point: Point, swaths: &[Swath]) -> Vec<usize> {
    if indices.is_empty() {
        return Vec::new();
    }

    let left_dist = endpoint_distance_to_swath(start_point, &swaths[indices[0]]);
    let right_dist = endpoint_distance_to_swath(start_point, &swaths[*indices.last().unwrap()]);
    let mut left = 0usize;
    let mut right = indices.len() - 1;
    let mut take_left = left_dist <= right_dist;
    let mut out = Vec::with_capacity(indices.len());

    while left <= right {
        if take_left {
            out.push(indices[left]);
            left += 1;
        } else {
            out.push(indices[right]);
            if right == 0 {
                break;
            }
            right -= 1;
        }
        take_left = !take_left;
    }

    out
}

fn endpoint_distance_to_swath(point: Point, swath: &Swath) -> f64 {
    point_distance(point, swath.head()).min(point_distance(point, swath.tail()))
}

fn maybe_reverse_for_start(traversal: &mut [Swath], start_point: Point) {
    if traversal.len() < 2 {
        return;
    }
    let first_dist = endpoint_distance_to_swath(start_point, &traversal[0]);
    let last_dist = endpoint_distance_to_swath(start_point, traversal.last().unwrap());
    if last_dist + 1e-9 < first_dist {
        traversal.reverse();
        for swath in traversal.iter_mut() {
            swath.swap_direction();
        }
    }
}

fn improve_adjacent_swaps(
    mut traversal: Vec<Swath>,
    start_point: Point,
    passes: usize,
) -> Vec<Swath> {
    for _ in 0..passes {
        let mut changed = false;
        for i in 0..traversal.len().saturating_sub(1) {
            let current_score = deadhead_distance(start_point, &traversal);
            let mut candidate = traversal.clone();
            candidate.swap(i, i + 1);
            orient_greedily_in_place(start_point, &mut candidate);
            let candidate_score = deadhead_distance(start_point, &candidate);
            if candidate_score + 1e-9 < current_score {
                traversal = candidate;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    traversal
}

fn orient_greedily_in_place(start_point: Point, traversal: &mut [Swath]) {
    let mut current = start_point;
    for swath in traversal.iter_mut() {
        let head_dist = point_distance(current, swath.head());
        let tail_dist = point_distance(current, swath.tail());
        if tail_dist < head_dist {
            swath.swap_direction();
        }
        current = swath.tail();
    }
}

fn deadhead_distance(start_point: Point, traversal: &[Swath]) -> f64 {
    if traversal.is_empty() {
        return 0.0;
    }
    let mut total = point_distance(start_point, traversal[0].head());
    for pair in traversal.windows(2) {
        total += point_distance(pair[0].tail(), pair[1].head());
    }
    total
}
