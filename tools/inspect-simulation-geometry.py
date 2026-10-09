#!/usr/bin/env python3
"""Check delivered world geometry and the entire fixed X500 inspection route.

Reads collision meshes, all maize instances and the actual heightmap. A
conservative rectangular takeoff column includes the vehicle, waypoint error
and a margin. Cruise clearance uses the maximum of every delivered collision
and maize visual, so it also covers intermediate segments and return.
Unsupported geometry fails explicitly; the check never changes an asset.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


def values(text, default):
    return np.fromstring(text, sep=' ') if text else np.array(default, dtype=float)


def transform(pose):
    x,y,z,r,p,yaw = pose
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    result = np.eye(4)
    result[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                    [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    result[:3,3] = [x,y,z]
    return result


def apply(points, matrix):
    return points @ matrix[:3,:3].T + matrix[:3,3]


def collada(path):
    root = ET.parse(path).getroot()
    for element in root.iter():
        element.tag = element.tag.split('}')[-1]
    if root.findtext('asset/up_axis') != 'Z_UP' or float(root.find('asset/unit').get('meter')) != 1:
        raise ValueError(f'Unsupported Collada coordinate system: {path}')
    geometries = {}
    for geometry in root.findall('library_geometries/geometry'):
        mesh = geometry.find('mesh')
        sources = {source.get('id'): np.fromstring(source.findtext('float_array'),sep=' ').reshape(
            -1,int(source.find('technique_common/accessor').get('stride','1')))
            for source in mesh.findall('source')}
        vertices = {vertex.get('id'):vertex.find("input[@semantic='POSITION']").get('source')[1:]
                    for vertex in mesh.findall('vertices')}
        if mesh.find('polylist') is not None or mesh.find('polygons') is not None:
            raise ValueError(f'Untriangulated Collada: {path}')
        pieces = []
        for primitive in mesh.findall('triangles'):
            inputs = primitive.findall('input')
            width = max(int(item.get('offset')) for item in inputs)+1
            vertex = next(item for item in inputs if item.get('semantic')=='VERTEX')
            indexes = np.fromstring(primitive.findtext('p'),sep=' ',dtype=int).reshape(-1,width)
            points = sources[vertices[vertex.get('source')[1:]]]
            pieces.append((primitive.get('material',''),points[indexes[:,int(vertex.get('offset'))]].reshape(-1,3,3)))
        geometries[geometry.get('id')] = pieces
    result = []
    def walk(node, parent):
        matrix = parent.copy()
        for child in node:
            if child.tag=='matrix': matrix = matrix @ np.fromstring(child.text,sep=' ').reshape(4,4)
            elif child.tag in {'translate','rotate','scale','lookat','skew'}:
                raise ValueError(f'Unsupported Collada transform {child.tag}: {path}')
        for instance in node.findall('instance_geometry'):
            result.extend((material,apply(points,matrix)) for material,points in geometries[instance.get('url')[1:]])
        for child in node.findall('node'): walk(child,matrix)
    for node in root.findall('library_visual_scenes/visual_scene/node'): walk(node,np.eye(4))
    if not result: raise ValueError(f'No mesh triangles: {path}')
    return result


def stl(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
    if len(data)==84+50*count:
        dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        return np.frombuffer(data,offset=84,count=count,dtype=dtype)['vertices'].astype(float)
    points = [values(line.strip()[7:],[]) for line in data.decode().splitlines() if line.strip().startswith('vertex ')]
    return np.array(points).reshape(-1,3,3)


def elevation(triangles, xy):
    a,b,c = triangles[:,0],triangles[:,1],triangles[:,2]
    d = (b[:,1]-c[:,1])*(a[:,0]-c[:,0])+(c[:,0]-b[:,0])*(a[:,1]-c[:,1])
    valid = abs(d)>1e-12
    a,b,c,d = a[valid],b[valid],c[valid],d[valid]
    u = ((b[:,1]-c[:,1])*(xy[0]-c[:,0])+(c[:,0]-b[:,0])*(xy[1]-c[:,1]))/d
    v = ((c[:,1]-a[:,1])*(xy[0]-c[:,0])+(a[:,0]-c[:,0])*(xy[1]-c[:,1]))/d
    inside = (u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)
    z = (u*a[:,2]+v*b[:,2]+(1-u-v)*c[:,2])[inside]
    if not len(z): raise ValueError(f'No terrain under {xy}')
    return float(z.max())


def footprint_elevations(triangles, xy, radius=.5):
    """Clip terrain triangles to the complete vehicle footprint in world XY."""
    near=np.all(triangles.min(axis=1)[:,:2]<=xy+radius,axis=1)&np.all(triangles.max(axis=1)[:,:2]>=xy-radius,axis=1)
    heights=[]
    for triangle in triangles[near]:
        polygon=list(triangle)
        for axis,limit,direction in [(0,xy[0]-radius,1),(0,xy[0]+radius,-1),
                                     (1,xy[1]-radius,1),(1,xy[1]+radius,-1)]:
            clipped=[]
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                inside_a=(a[axis]-limit)*direction>=0
                inside_b=(b[axis]-limit)*direction>=0
                if inside_a: clipped.append(a)
                if inside_a!=inside_b:
                    clipped.append(a+(b-a)*(limit-a[axis])/(b[axis]-a[axis]))
            polygon=clipped
            if not polygon: break
        heights.extend(point[2] for point in polygon)
    if not heights: raise ValueError(f'No terrain under footprint {xy}')
    return [float(min(heights)),float(max(heights))]


class Inspection:
    def __init__(self, clearpath, maize, maize_models):
        self.clearpath,self.maize,self.maize_models = clearpath,maize,maize_models
        self.hashes = {}

    def register(self,path):
        if str(path) not in self.hashes:
            self.hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        return path

    def air_collision_boxes(self, models):
        """Read the delivered X500 and camera collisions, including leg depth."""
        boxes = []
        def visit(name, parent):
            model = ET.parse(self.register(models/name/'model.sdf')).getroot().find('model')
            matrix = parent @ transform(values(model.findtext('pose'), [0]*6))
            for include in model.findall('include'):
                visit(include.findtext('uri').removeprefix('model://'),
                      matrix @ transform(values(include.findtext('pose'), [0]*6)))
            for link in model.findall('link'):
                pose = link.find('pose')
                if pose is not None and pose.get('relative_to') not in {None, '__model__', 'base_link'}:
                    raise ValueError(f'Unsupported X500 link frame: {name}/{link.get("name")}')
                link_matrix = matrix @ transform(values(link.findtext('pose'), [0]*6))
                for collision in link.findall('collision'):
                    box = collision.find('geometry/box')
                    if box is None:
                        raise ValueError(f'Unsupported X500 collision: {name}/{collision.get("name")}')
                    half = values(box.findtext('size'), []) / 2
                    corners = np.array([[x,y,z] for x in [-half[0],half[0]]
                        for y in [-half[1],half[1]] for z in [-half[2],half[2]]])
                    boxes.append(apply(corners, link_matrix @ transform(
                        values(collision.findtext('pose'), [0]*6))))
        visit('x500_depth', np.eye(4))
        return boxes

    def world_meshes(self,site):
        world_path = self.register(self.clearpath/'worlds'/f'{site}.sdf')
        world = ET.parse(world_path).getroot().find('world')
        terrain,obstacles = [],[]
        for model in world.findall('model'):
            model_pose = transform(values(model.findtext('pose'),[0]*6))
            for link in model.findall('link'):
                link_pose = model_pose @ transform(values(link.findtext('pose'),[0]*6))
                for collision in link.findall('collision'):
                    matrix = link_pose @ transform(values(collision.findtext('pose'),[0]*6))
                    mesh = collision.find('geometry/mesh')
                    if mesh is None: raise ValueError(f'Unsupported world collision: {site}/{model.get("name")}')
                    uri = mesh.findtext('uri').strip()
                    if not uri.startswith('model://'): raise ValueError(uri)
                    path = self.register(self.clearpath/'meshes'/uri[8:])
                    pieces = collada(path) if path.suffix=='.dae' else [('',stl(path))]
                    scale = values(mesh.findtext('scale'),[1,1,1])
                    for material,points in pieces:
                        points = apply(points*scale,matrix)
                        is_terrain = model.get('name')==site and ('HEIGHT' in material or 'Ground' in material)
                        (terrain if is_terrain else obstacles).append(points)
        return np.concatenate(terrain),np.concatenate(obstacles),world

    def maize_geometry(self):
        world = ET.parse(self.register(self.maize/'generated.world')).getroot().find('world')
        heightmap = world.find('model/link/collision/geometry/heightmap')
        size = values(heightmap.findtext('size'),[])
        image_path = self.register(self.maize/Path(heightmap.findtext('uri')).name)
        pixels = np.array(Image.open(image_path).convert('L'),dtype=float)
        if pixels.shape[0]!=pixels.shape[1] or size[2]<=0 or pixels.max()==0:
            raise ValueError('Unsupported maize heightmap dimensions or elevation')
        sampling = int(heightmap.findtext('sampling') or '2')
        # Harmonic DART CustomHeightmapShape uses size.z / MaxElevation().
        # ImageHeightmap's inherited MinElevation is zero, not the darkest
        # pixel. DART's Bullet collision detector then flips the image rows.
        heights = pixels[::-1,:]*size[2]/pixels.max()
        vertex_count = (len(heights)-1)*sampling+1
        dx,dy = size[0]*sampling/vertex_count,size[1]*sampling/vertex_count
        gy,gx = np.gradient(heights,dy,dx)
        slopes = np.degrees(np.arctan(np.hypot(gx,gy)))
        def pixel_coordinate(xy):
            return np.array(xy)/[dx,dy]+(len(heights)-1)/2
        def sample(xy):
            point=pixel_coordinate(xy)
            ix,iy=np.floor(point).astype(int)
            if not (0<=ix<len(heights)-1 and 0<=iy<len(heights)-1): raise ValueError(f'Outside heightmap {xy}')
            # Both Bullet triangle diagonals lie inside this corner envelope.
            # Return a lower bound, never an optimistic interpolated height.
            return float(heights[iy:iy+2,ix:ix+2].min())
        def footprint_range(xy, radius=.5):
            lower=np.floor(pixel_coordinate(np.array(xy)-radius)).astype(int)
            upper=np.ceil(pixel_coordinate(np.array(xy)+radius)).astype(int)
            if np.any(lower<0) or np.any(upper>=len(heights)): raise ValueError(f'Footprint outside heightmap {xy}')
            patch=heights[lower[1]:upper[1]+1,lower[0]:upper[0]+1]
            return [float(patch.min()),float(patch.max())]
        bounds = []
        visual_max=collision_max=-math.inf
        plant_cache={}
        for include in world.findall('include'):
            name = include.findtext('uri').removeprefix('model://')
            if name not in {'maize_01','maize_02'}: raise ValueError(f'Unexpected maize asset {name}')
            model = ET.parse(self.register(self.maize_models/name/'model.sdf')).getroot().find('model')
            pose = transform(values(include.findtext('pose'),[0]*6))
            if name not in plant_cache:
                visual,collision=[],[]
                for link in model.findall('link'):
                    link_pose=transform(values(model.findtext('pose'),[0]*6)) @ transform(values(link.findtext('pose'),[0]*6))
                    for kind,pieces in [('visual',visual),('collision',collision)]:
                        for item in link.findall(kind):
                            mesh=item.find('geometry/mesh')
                            box=item.find('geometry/box')
                            if mesh is not None:
                                path=self.register(self.maize_models/mesh.findtext('uri').removeprefix('model://'))
                                p=np.concatenate([p for _,p in collada(path)]).reshape(-1,3)*values(mesh.findtext('scale'),[1,1,1])
                            elif box is not None:
                                half=values(box.findtext('size'),[])/2
                                p=np.array([[x,y,z] for x in [-half[0],half[0]]
                                    for y in [-half[1],half[1]] for z in [-half[2],half[2]]])
                            else: raise ValueError(f'Unsupported maize {kind}: {name}')
                            pieces.append(apply(p,link_pose @ transform(values(item.findtext('pose'),[0]*6))))
                # Retain the source model offset conservatively even though
                # an SDF include pose overrides its model pose.
                plant_cache[name]=(np.concatenate(visual),np.concatenate(collision))
            visual,collision=[apply(p,pose) for p in plant_cache[name]]
            visual_max=max(visual_max,float(visual[:,2].max()))
            collision_max=max(collision_max,float(collision[:,2].max()))
            p=np.concatenate([visual,collision])
            bounds.append([p.min(axis=0),p.max(axis=0)])
        bounds=np.array(bounds)
        summary={'terrain_z_range':[float(heights.min()),float(heights.max())],
            'terrain_pixel_slope_degrees':{'max':float(slopes.max()),'percentile95':float(np.percentile(slopes,95))},
            'heightmap_size_m':size.tolist(),'heightmap_resolution':list(heights.shape),
            'heightmap_physics':{'sampling':sampling,'vertex_count_per_axis':vertex_count,
                'z_conversion':'pixel / maximum_pixel * size.z (MinElevation=0)',
                'row_direction':'Bullet flips ImageHeightmap rows',
                'spawn_elevation':'lower bound of enclosing pixel corners'},
            'plant_count':len(bounds),'plant_visual_z_max':visual_max,
            'plant_collision_z_max':collision_max,
            'plant_collision_model':'original rigid 0.02 x 0.02 x 0.4 m boxes; not flexible crop physics'}
        return summary,bounds,sample,footprint_range,world


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clearpath-share',type=Path,default=Path('/opt/ros/jazzy/share/clearpath_gz'))
    parser.add_argument('--maize-world',type=Path,default=Path('/opt/greenhouse_sim/worlds/maize_field'))
    parser.add_argument('--maize-models',type=Path,default=Path('/opt/maize_field/share/virtual_maize_field/models'))
    parser.add_argument('--px4-models',type=Path,default=Path('/opt/px4/Tools/simulation/gz/models'))
    parser.add_argument('--air-config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    inspection=Inspection(args.clearpath_share,args.maize_world,args.maize_models)
    sites=yaml.safe_load(inspection.register(args.air_config).read_text())
    air_boxes = inspection.air_collision_boxes(args.px4_models)
    report={'schema':1,'air_config_sha256':inspection.hashes[str(args.air_config)],
        'coordinate_conversion':'world ENU: x=spawn.x+waypoint.y, y=spawn.y+waypoint.x; altitude relative to settled home',
        'envelope':{'vehicle_radius_m':.5,'xy_error_m':1.5,'horizontal_margin_m':.5,
                    'vertical_half_extent_m':.5,'altitude_error_m':.6,'vertical_margin_m':.5},
        'limitations':['Static asset geometry check, not online 3D avoidance or terrain following',
            'Normal takeoff and landing ground contact is expected; maize is a rigid model',
            'Triangle/pixel slopes describe the asset, not validated traversability limits'], 'sites':{}}
    for site,config in sites.items():
        spawn=np.array(config['spawn'][:3]); radius=2.5
        if site=='maize_field':
            summary,bounds,sample,footprint_range,world=inspection.maize_geometry()
            global_max=max(summary['terrain_z_range'][1],summary['plant_visual_z_max'],summary['plant_collision_z_max'])
            near=np.all(bounds[:,0,:2]<=spawn[:2]+radius,axis=1)&np.all(bounds[:,1,:2]>=spawn[:2]-radius,axis=1)
            near_count=int(near.sum())
        else:
            terrain,obstacles,world=inspection.world_meshes(site)
            normals=np.cross(terrain[:,1]-terrain[:,0],terrain[:,2]-terrain[:,0]); lengths=np.linalg.norm(normals,axis=1)
            slopes=np.degrees(np.arccos(np.clip(abs(normals[lengths>1e-12,2])/lengths[lengths>1e-12],0,1)))
            summary={'terrain_z_range':[float(terrain[:,:,2].min()),float(terrain[:,:,2].max())],
                'terrain_face_slope_degrees':{'max':float(slopes.max()),'percentile95':float(np.percentile(slopes,95))},
                'terrain_triangles':len(terrain),'obstacle_triangles':len(obstacles)}
            global_max=max(float(terrain[:,:,2].max()),float(obstacles[:,:,2].max()))
            near=np.all(obstacles.min(axis=1)[:,:2]<=spawn[:2]+radius,axis=1)&np.all(obstacles.max(axis=1)[:,:2]>=spawn[:2]-radius,axis=1)
            near_count=int(near.sum())
            sample=lambda xy:elevation(terrain,xy)
            footprint_range=lambda xy,radius=.5:footprint_elevations(terrain,xy,radius)
        ground=footprint_range(spawn[:2])
        collision_clearances = []
        for box in air_boxes:
            placed = apply(box, transform([*spawn, 0, 0, config['spawn'][3]]))
            lower, upper = placed.min(axis=0), placed.max(axis=0)
            center = (lower[:2]+upper[:2])/2
            box_radius = float(max(upper[:2]-lower[:2])/2)
            collision_clearances.append(float(lower[2]-max(footprint_range(center,box_radius))))
        spawn_clearance = min(collision_clearances)
        home_lower=ground[0]
        route=[spawn[:2].tolist()]+[[float(spawn[0]+w['y']),float(spawn[1]+w['x'])] for w in config['waypoints']]+[spawn[:2].tolist()]
        altitude=float(config['cruise_altitude'])
        vertical_clearance=home_lower+altitude-global_max
        passed=(near_count==0 and vertical_clearance>=1.6
                and all(float(w['z'])==altitude for w in config['waypoints'])
                and max(ground)-min(ground)<=.3 and spawn_clearance>=0)
        physics=world.find('physics')
        summary.update(air_spawn=config['spawn'],ground_under_spawn_m=sample(spawn[:2]),
            ground_under_vehicle_range_m=[min(ground),max(ground)],
            spawn_collision_clearance_m=spawn_clearance,
            world_max_geometry_z_m=global_max,physics_step_s=float(physics.findtext('max_step_size')),
            gravity=world.findtext('gravity') or physics.findtext('gravity') or '0 0 -9.8 (SDF default)',
            air_clearance={'result':'pass' if passed else 'fail','takeoff_column_half_width_m':radius,
                'obstacle_candidates_in_entire_vertical_column':near_count,
                'cruise_altitude_above_home_m':altitude,'conservative_vertical_clearance_m':vertical_clearance,
                'required_vertical_clearance_m':1.6,'world_xy_route_including_return':route,
                'checked_segments':len(route)-1,'takeoff_and_landing':'same obstacle-free vertical column'})
        report['sites'][site]=summary
    report['asset_sha256']=inspection.hashes
    report['result']='pass' if all(s['air_clearance']['result']=='pass' for s in report['sites'].values()) else 'fail'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'result':report['result'],'output':str(args.output),
        'sites':{name:value['air_clearance'] for name,value in report['sites'].items()}}))
    return 0 if report['result']=='pass' else 1


if __name__=='__main__': raise SystemExit(main())
