use std::{mem::swap, ops::Add};

fn main() {
    const SIZE: usize = 32;
    let forces = vec![
        Force {
            position: Vec3 {
                x: 4.0,
                y: 6.0,
                z: 0.0,
            },
            force: 2.0,
        },
        Force {
            position: Vec3 {
                x: -4.0,
                y: 6.0,
                z: 0.0,
            },
            force: 2.5,
        },
        Force {
            position: Vec3 {
                x: 4.0,
                y: -6.0,
                z: -4.0,
            },
            force: 2.5,
        },
    ];
    let mut domain = Domain {
        from: Vec3 {
            x: -16.0,
            y: -16.0,
            z: -16.0,
        },
        to: Vec3 {
            x: 16.0,
            y: 16.0,
            z: 16.0,
        },
        surface_weight: 1.0,
        width: SIZE,
        height: SIZE,
        depth: SIZE,
        meshes: Vec::default(),
    };
    domain.march_tetrahedras(&weight_function, &refine_function_linear, &forces);
    domain.export_to_bpy();
}

struct Force {
    position: Vec3,
    force: f64,
}

fn weight_function(position: Vec3, data: &Vec<Force>) -> f64 {
    let mut total_weight = 0.0;
    for force in data {
        let dx = position.x - force.position.x;
        let dy = position.y - force.position.y;
        let dz = position.z - force.position.z;
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();
        let weight = force.force / distance;
        total_weight += weight;
    }
    total_weight
}

fn refine_function_center<WEIGHT, DATA>(
    v1: Vec3,
    v2: Vec3,
    _weight_function: &WEIGHT,
    _weight_user_data: &DATA,
    _surface_weight: f64,
) -> Vec3
where
    WEIGHT: Fn(Vec3, &DATA) -> f64,
{
    Vec3 {
        x: (v1.x + v2.x) * 0.5,
        y: (v1.y + v2.y) * 0.5,
        z: (v1.z + v2.z) * 0.5,
    }
}

fn refine_function_linear<WEIGHT, DATA>(
    v1: Vec3,
    v2: Vec3,
    weight_function: &WEIGHT,
    weight_user_data: &DATA,
    surface_weight: f64,
) -> Vec3
where
    WEIGHT: Fn(Vec3, &DATA) -> f64,
{
    let mut pos_left = v1;
    let mut pos_right = v2;
    let w_left = weight_function(pos_left, weight_user_data);
    let w_right = weight_function(pos_right, weight_user_data);
    if w_left > w_right {
        swap(&mut pos_left, &mut pos_right);
    }

    let mut pos_center = pos_left;
    for _ in 0..8 {
        pos_center = refine_function_center(
            pos_left,
            pos_right,
            weight_function,
            weight_user_data,
            surface_weight,
        );
        let w_center = weight_function(pos_center, weight_user_data);
        if w_center < surface_weight {
            pos_left = pos_center;
        } else {
            pos_right = pos_center;
        }
    }

    pos_center
}

/// Tetrahedra has 4 verts and 4 faces. The first vert is considered the top, the others part of the bottom.
///

/// Map each tetrahedra vertex masks to the edges that will be based for the faces.
/// Although there are 16 possible vert maps, the last 8 are the inverse of the first 8 so we only need to store 8 of them.
/// When using the inverse the edge2 and edge3 should be inversed as well to ensure correct "normals".
const TETRADEDRA_VERTMASK_TO_EDGES: [[isize; 6]; 8] = [
    [-1, -1, -1, -1, -1, -1], // 0000/1111
    [0, 1, 2, -1, -1, -1],    // 0001/1110
    [0, 5, 3, -1, -1, -1],    // 0010/1101
    [1, 2, 3, 3, 2, 5],       // 0011/1100
    [1, 3, 4, -1, -1, -1],    // 0100/1011
    [4, 2, 3, 3, 2, 0],       // 0101/1010
    [1, 0, 4, 4, 0, 5],       // 0110/1001
    [2, 5, 4, -1, -1, -1],    // 0111/1000
];

/// Ordering of verts inside a grid block
const GRID_TO_VERT_OFFSETS: [IVec3; 8] = [
    IVec3 { x: 0, y: 0, z: 0 },
    IVec3 { x: 1, y: 0, z: 0 },
    IVec3 { x: 1, y: 1, z: 0 },
    IVec3 { x: 0, y: 1, z: 0 },
    IVec3 { x: 0, y: 0, z: 1 },
    IVec3 { x: 1, y: 0, z: 1 },
    IVec3 { x: 1, y: 1, z: 1 },
    IVec3 { x: 0, y: 1, z: 1 },
];
/// Split a grid into 5 tetrahedras.
const GRID_TO_TETRAHEDRA_VERTICES: [[usize; 4]; 5] = [
    [0, 2, 7, 5],
    [1, 0, 5, 2],
    [3, 2, 7, 0],
    [4, 0, 7, 5],
    [6, 2, 5, 7],
];
const TETRAHEDRA_EDGES_TO_VERT_OFFSETS: [[usize; 2]; 6] =
    [[0, 1], [0, 2], [0, 3], [1, 2], [2, 3], [3, 1]];

#[derive(Copy, Clone, Debug)]
struct IVec3 {
    x: i32,
    y: i32,
    z: i32,
}

impl Add<IVec3> for IVec3 {
    type Output = IVec3;

    fn add(self, rhs: IVec3) -> Self::Output {
        IVec3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct Vec3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug)]
struct Face {
    v1: usize,
    v2: usize,
    v3: usize,
}
#[derive(Debug)]
struct Edge {
    v1: usize,
    v2: usize,
}

#[derive(Debug, Default)]
struct Mesh {
    verts: Vec<Vec3>,
    faces: Vec<Face>,
    edges: Vec<Edge>,
}

#[derive(Debug)]
struct Domain {
    from: Vec3,
    to: Vec3,

    surface_weight: f64,
    width: usize,
    height: usize,
    depth: usize,

    meshes: Vec<Mesh>,
}

impl Domain {
    fn vertex_grid_size(&self) -> IVec3 {
        IVec3 {
            x: self.width as i32 + 1,
            y: self.height as i32 + 1,
            z: self.depth as i32 + 1,
        }
    }

    fn vertex_position(&self, vertex_grid_position: IVec3) -> Vec3 {
        Vec3 {
            x: self.from.x
                + vertex_grid_position.x as f64 * (self.to.x - self.from.x) / self.width as f64,
            y: self.from.y
                + vertex_grid_position.y as f64 * (self.to.y - self.from.y) / self.height as f64,
            z: self.from.z
                + vertex_grid_position.z as f64 * (self.to.z - self.from.z) / self.depth as f64,
        }
    }
}

fn get_vert_offsets(cell_pos: IVec3) -> ([IVec3; 8], bool) {
    let flip_x = cell_pos.x.abs() & 1 != 0;
    let flip_y = cell_pos.y.abs() & 1 != 0;
    let flip_z = cell_pos.z.abs() & 1 != 0;
    let grid_inverse = [flip_x, flip_y, flip_z].iter().filter(|v| **v).count() & 1 != 0;

    let mut result = GRID_TO_VERT_OFFSETS;

    for index in 0..8 {
        if flip_x {
            result[index].x = 1 - result[index].x;
        }
        if flip_y {
            result[index].y = 1 - result[index].y;
        }
        if flip_z {
            result[index].z = 1 - result[index].z;
        }
    }
    (result, grid_inverse)
}

impl Domain {
    fn march_tetrahedras<WEIGHT, REFINE, DATA>(
        &mut self,
        weight_function: &WEIGHT,
        refine_function: &REFINE,
        weight_user_data: &DATA,
    ) where
        WEIGHT: Fn(Vec3, &DATA) -> f64,
        DATA: Sized,
        REFINE: Fn(Vec3, Vec3, &WEIGHT, &DATA, f64) -> Vec3,
    {
        let mut mesh = Mesh::default();
        let max_cell_position = self.vertex_grid_size();
        for x in 0..max_cell_position.x {
            for y in 0..max_cell_position.y {
                for z in 0..max_cell_position.z {
                    let cell_pos = IVec3 { x, y, z };
                    let (grid_to_verts_offsets, grid_inverse) = get_vert_offsets(cell_pos);
                    let vert_positions = grid_to_verts_offsets
                        .iter()
                        .map(|offset| cell_pos + *offset)
                        .map(|grid_position| self.vertex_position(grid_position))
                        .collect::<Vec<Vec3>>();

                    let vert_is_inside = vert_positions
                        .iter()
                        .map(|vert_position| weight_function(*vert_position, weight_user_data))
                        .map(|weight| weight > self.surface_weight)
                        .collect::<Vec<bool>>();
                    for tetrahedron_indices in GRID_TO_TETRAHEDRA_VERTICES {
                        // determine vert mask + inverse
                        let mut mask = 0;
                        for index in 0..tetrahedron_indices.len() {
                            let index_mask = 1 << index;
                            if vert_is_inside[tetrahedron_indices[index]] {
                                mask |= index_mask;
                            }
                        }
                        let compressed_mask = if mask > 7 { 15 - mask } else { mask } as usize;
                        let inversed_mask = (mask > 7) != grid_inverse;
                        for face_index in 0..2 {
                            let e1 = TETRADEDRA_VERTMASK_TO_EDGES[compressed_mask][face_index * 3];
                            let e2 =
                                TETRADEDRA_VERTMASK_TO_EDGES[compressed_mask][face_index * 3 + 1];
                            let e3 =
                                TETRADEDRA_VERTMASK_TO_EDGES[compressed_mask][face_index * 3 + 2];
                            if e1 == -1 {
                                // No faces left to add for this tetrahedra.
                                break;
                            }
                            let face_vert_start_index = mesh.verts.len();
                            mesh.faces.push(Face {
                                v1: face_vert_start_index,
                                v2: face_vert_start_index + if inversed_mask { 2 } else { 1 },
                                v3: face_vert_start_index + if inversed_mask { 1 } else { 2 },
                            });
                            mesh.edges.push(Edge {
                                v1: face_vert_start_index,
                                v2: face_vert_start_index + 1,
                            });
                            mesh.edges.push(Edge {
                                v1: face_vert_start_index + 1,
                                v2: face_vert_start_index + 2,
                            });
                            mesh.edges.push(Edge {
                                v1: face_vert_start_index + 2,
                                v2: face_vert_start_index,
                            });
                            for edge_index in [e1, e2, e3] {
                                let edge_vert_offs =
                                    TETRAHEDRA_EDGES_TO_VERT_OFFSETS[edge_index as usize];
                                let vert_offs_1 = edge_vert_offs[0];
                                let vert_offs_2 = edge_vert_offs[1];
                                let vert_pos_1 = vert_positions[tetrahedron_indices[vert_offs_1]];
                                let vert_pos_2 = vert_positions[tetrahedron_indices[vert_offs_2]];
                                let edge_pos = refine_function(
                                    vert_pos_1,
                                    vert_pos_2,
                                    weight_function,
                                    weight_user_data,
                                    self.surface_weight,
                                );
                                mesh.verts.push(edge_pos);
                            }
                        }
                    }
                }
            }
        }
        self.meshes.push(mesh);
    }

    fn export_to_bpy(&self) {
        println!("import bpy");
        println!("");
        for mesh in &self.meshes {
            mesh.export_to_bpy("Marching");
        }
    }
}

impl Mesh {
    fn export_to_bpy(&self, name: &str) {
        println!("verts = [");
        for vert in &self.verts {
            println!("  ({:8}, {:8}, {:8}),", vert.x, vert.y, vert.z);
        }
        println!("]");
        println!("edges = [");
        for edge in &self.edges {
            println!("  ({:4}, {:4}),", edge.v1, edge.v2);
        }
        println!("]");
        println!("faces = [");
        for face in &self.faces {
            println!("  ({:4}, {:4}, {:4}),", face.v1, face.v2, face.v3);
        }
        println!("]");
        println!("new_mesh = bpy.data.meshes.new('{name}')");
        println!("new_mesh.from_pydata(verts, edges, faces)");
        println!("");
        println!("new_object = bpy.data.objects.new('{name}', new_mesh)");
        println!("bpy.context.scene.collection.objects.link(new_object)");
    }
}
