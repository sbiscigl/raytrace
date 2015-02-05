/*
NOTE: This is the file you will need to begin working with
You will need to implement the RayTrace::CalculatePixel () function

This file defines the following:
RayTrace Class
*/

#ifndef RAYTRACE_H
#define RAYTRACE_H

#include <stdio.h>
#include <stdlib.h>

#include "Utils.h"
#include <math.h>

#include <limits.h>
#include <float.h>

#define EPSILON .001

/*
RayTrace Class - The class containing the function you will need to implement

This is the class with the function you need to implement
*/

//DECLARE STRUCT FOR SPHERE INTERSECTION
struct intersection{
	float t; //distance to intersection point
    Vector normal, position;
	bool is_hit; //says if intersection is hit
	SceneObject *object;
};

//FUNCTION TO RETURN INTERSECTION OF SPHERE
void sphere_intersection(Vector ray, SceneSphere *sphere, Vector camerapos, intersection *intertemp){
	float t0, t1;
	float a = 1;
	float b = 2 * ((ray.x * (camerapos.x - sphere->center.x) + ray.y * (camerapos.y - sphere->center.y) + ray.z * (camerapos.z - sphere->center.z)));
	float c = ((camerapos.x - sphere->center.x)*(camerapos.x - sphere->center.x)) + ((camerapos.y - sphere->center.y)*(camerapos.y - sphere->center.y)) + ((camerapos.z - sphere->center.z)*(camerapos.z - sphere->center.z)) - sphere->radius*sphere->radius;
	//Find discriminant
	float disc = b * b - 4 * a * c;
	// if discriminant is negative there are no real roots, so return 
	// false as ray misses sphere
	if (disc < 0){
		intertemp->is_hit = FALSE;
		return;
	}
	// compute q as described above
	float distSqrt = sqrtf(disc);

	t0 = (-b - distSqrt)/2.0;
	t1 = (-b + distSqrt)/2.0;

	// if t0 is less than zero, the intersection point is at t1
	if (t0 > 0)
	{
		intertemp->is_hit = TRUE;
		intertemp->t = t0;
		intertemp->object = sphere;
		intertemp->position = camerapos + (ray*t0);
	}
	// else the intersection point is at t0
	else
	{
		intertemp->is_hit = TRUE;
		intertemp->t = t1;
		intertemp->object = sphere;
		intertemp->position = camerapos + (ray*t1);
	}
	intertemp->normal = (intertemp->position - sphere->center).Normalize();
	intertemp->object = sphere;
}

//FUNCTION FOR TRIANGLE INTERSECTION
void triangle_intersection(Vector ray, SceneTriangle *trianlge, Vector camerapos, intersection *intertemp){
	Vector tvec, pvec, qvec;
	float det, inv_det;
	float u, v, t;

	Vector edge1 = trianlge->vertex[1] - trianlge->vertex[0];
	Vector edge2 = trianlge->vertex[2] - trianlge->vertex[0];

	pvec = ray.Cross(edge2);
	det = edge1.Dot(pvec);

	if (det > -EPSILON && det < EPSILON){
		intertemp->is_hit = FALSE;
		return;
	}

	inv_det = 1.0 / det;

	tvec = camerapos - trianlge->vertex[0];

	u = tvec.Dot(pvec) * inv_det;

	if (u < 0.0 || u > 1.0){
		intertemp->is_hit = FALSE;
		return;
	}

	qvec = tvec.Cross(edge1);

	v = ray.Dot(qvec) * inv_det;

	if (v < 0.0 || (u + v) > 1.0){
		intertemp->is_hit = FALSE;
		return;
	}

	t = edge2.Dot(qvec) * inv_det;

	intertemp->t = t;
	intertemp->is_hit = TRUE;
	intertemp->normal = edge1.Cross(edge2);
	intertemp->object = trianlge;
	intertemp->position = camerapos + ray*t;

}

void model_intersection(Vector ray, SceneModel *model, Vector camerapos, intersection *intertemp){
	//Intersection holder
	intersection inter_hold;

	intertemp->t = 999999;
	intertemp->is_hit = FALSE;

	int num_triangles = model->GetNumTriangles();
	for(int i = 0; i < num_triangles; i++){

		SceneTriangle *tri = (SceneTriangle*)model->GetTriangle(i);
		triangle_intersection(ray, tri, camerapos, &inter_hold);

		if(inter_hold.is_hit == TRUE){
			if(inter_hold.t < intertemp->t){
				//*intertemp = inter_hold;
				intertemp->is_hit = inter_hold.is_hit;
				intertemp->normal = inter_hold.normal;
				intertemp->object = inter_hold.object;
				intertemp->position = inter_hold.position;
				intertemp->t = inter_hold.t;
			}
		}
	}

}



class RayTrace
{
public:
	/* - Scene Variable for the Scene Definition - */
	Scene m_Scene;

	// -- Constructors & Destructors --
	RayTrace (void) {}
	~RayTrace (void) {}

	// -- Main Functions --
	intersection get_intersection(Vector ray, Vector origin){
		intersection itemp, ifinal;
		ifinal.is_hit=FALSE;
		ifinal.t = FLT_MAX;

		int numObjs = m_Scene.GetNumObjects();
		for( int i=0; i<numObjs; i++){
			//DETERMINE OBJECT INTERSECTION
			if(m_Scene.GetObject(i)->IsSphere()){
				SceneSphere *s= (SceneSphere *)m_Scene.GetObject(i);
				//SPHERE INTERSECITON FUNCITON
				sphere_intersection(ray, s, origin, &itemp);
			}
			if(m_Scene.GetObject(i)->IsTriangle()){
				SceneTriangle *t= (SceneTriangle *)m_Scene.GetObject(i);
				triangle_intersection(ray, t, origin, &itemp);
			}
			if(m_Scene.GetObject(i)->IsModel()){
				SceneModel *m = (SceneModel *)m_Scene.GetObject(i);
				model_intersection(ray,m,origin,&itemp);
			}
			if(itemp.is_hit){
				if(itemp.t	< ifinal.t){
					ifinal  = itemp;
				}
			}
		}
		return ifinal;
	}

	// - CalculatePixel - Returns the Computed Pixel for that screen coordinate
	Vector CalculatePixel (int screenX, int screenY)
	{
		//DECLARE VARIBALE FOR FINAL PIXEL
		Vector final_pixel;

		if ((screenX < 0 || screenX > WINDOW_WIDTH - 1) ||
			(screenY < 0 || screenY > WINDOW_HEIGHT - 1))
		{
			// Off the screen, return black
			return Vector (0.0f, 0.0f, 0.0f);
		}

		//FIND SCENE CAMERA
		Camera camera = m_Scene.m_Camera;

		Vector cam_pos = m_Scene.m_Camera.position;
		Vector cam_tar = m_Scene.m_Camera.target;
		Vector cam_up  = m_Scene.m_Camera.up;

		float cam_fov  = m_Scene.m_Camera.fieldOfView;
		float cam_near = m_Scene.m_Camera.nearClip;


		//FIND COORDINAES OF THE DISPLAY REIGON
		float aspect_ratio = ((float)WINDOW_WIDTH/(float)WINDOW_HEIGHT);
		float height	   = 2.0*tan((cam_fov/2)*(3.14/180)) * cam_near;
		float half_height  = height/2.0;
		float half_width   = (aspect_ratio * height)/2.0;
		float width = half_width * 2.0;

		//CAMERA ORIENTATION
		Vector camera_z = (camera.GetTarget() - cam_pos).Normalize();
		Vector camera_y = cam_up.Normalize();
		Vector camera_x = camera_z.Cross(camera_y);

		Vector pixpos = cam_pos + (camera_z * cam_near) + (camera_x *((((width) / (float)WINDOW_WIDTH))*screenX - ((width/2.0)))) + (camera_y * ((((height) / (float)WINDOW_HEIGHT))*screenY - ((height/2.0))));

		Vector ray = (pixpos - cam_pos).Normalize();

		//FIND NUMBER OF OBJECTS IN SCENE
		int numObjs = m_Scene.GetNumObjects();
		//GET NUMBER OF LIGHTS IN SCENE
		int numLights = m_Scene.GetNumLights();

		intersection ifinal;

		ifinal = get_intersection(ray, cam_pos);

		Vector phong = m_Scene.GetBackground().color;
		Vector light;
		intersection inter_light, inter_light0;
		//IF INTERSECTION EXISTS
		if(ifinal.is_hit){
			//FOR EACH LIGHT IN THE SCENE
			for(int j=0; j < numLights; j++){
				Vector light_pos = m_Scene.GetLight(j).position;
				Vector light = (m_Scene.GetLight(j).position - ifinal.position).Normalize();
				//add this light contribrution to computed colour
				if(ifinal.object->IsSphere()){
					SceneSphere *f= (SceneSphere *)ifinal.object;

					Vector H = (light + (cam_pos - ifinal.position).Normalize()) * .5;
					SceneMaterial mat = m_Scene.GetMaterial(f->material);

					Vector diffuse = mat.diffuse * ifinal.normal.Dot(light) * m_Scene.GetLight(j).color;
					Vector ambient = m_Scene.GetBackground().ambientLight;
					Vector specular = mat.specular * pow(H.Dot(ifinal.normal), mat.shininess);

					phong = diffuse + ambient + specular;
				}
				if(ifinal.object->IsTriangle()){
					SceneTriangle *tf = (SceneTriangle*)ifinal.object;

					Vector H = (light + (cam_pos - ifinal.position).Normalize()) * .5;
					SceneMaterial mat = m_Scene.GetMaterial(tf->material[0]);

					Vector diffuse = mat.diffuse * ifinal.normal.Dot(light) * m_Scene.GetLight(j).color;
					Vector ambient = m_Scene.GetBackground().ambientLight;
					Vector specular = mat.specular * pow(H.Dot(ifinal.normal), mat.shininess);

					phong = diffuse + ambient + specular;
				}

				intersection shadow = get_intersection(light * -1, light_pos);
				if(shadow.is_hit){
					float delta = (shadow.position - ifinal.position).Magnitude();
					if(delta > EPSILON){
						phong = phong * .25;
					}
				}

				final_pixel = final_pixel + phong;
			}
			//final_pixel = phong;
		} else {
			final_pixel = phong;
		}

		return final_pixel;
	}


	//return final_pixel;
	//}

};

#endif // RAYTRACE_H
