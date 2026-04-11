use std::collections::HashSet;

use geo::Point;
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::{
    EdgeType, Graph, VertexId, add_edge_with_weight, add_vertex_with_property, num_edges,
    num_vertices,
};

use crate::core::{point_distance, segment_end, segment_length, segment_start};
use crate::field::{Swath, SwathType};

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
        if self.swaths.is_empty() {
            return;
        }

        let start_point = start_point.unwrap_or(segment_start(self.swaths[0].line));
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
                .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
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

        let mut ordered = Vec::with_capacity(traversal.len().saturating_mul(2));
        for (index, swath) in traversal.iter().cloned().enumerate() {
            ordered.push(swath.clone());
            if let Some(next) = traversal.get(index + 1) {
                let connection = Swath {
                    line: geo::Line::new(swath.tail().0, next.head().0),
                    uuid: format!("connection_{}_{}", swath.uuid, next.uuid),
                    r#type: SwathType::Connection,
                    finished: false,
                    bounding_box: crate::core::aabb_from_points(&[swath.tail(), next.head()]),
                    id: -1,
                    width: 0.0,
                    points: vec![swath.tail(), next.head()],
                };
                ordered.push(connection);
            }
        }

        self.swaths = ordered;
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

}
