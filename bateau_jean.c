#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

float WaterHeightAt(float x, float z, float t);

#define RHO 1000.0f
#define G 9.81f
#define L 8.0f
#define B 3.0f
#define V3(x, y, z) ((Vector3){ x, y, z })
#define ADD Vector3Add
#define SUB Vector3Subtract
#define MUL Vector3Scale
#define CROSS Vector3CrossProduct
#define ROT Vector3RotateByQuaternion
#define LEN Vector3Length
#define DOT Vector3DotProduct
#define NORM Vector3Normalize
#define QID QuaternionIdentity
#define V0 Vector3Zero
#define DRB_act rlDrawRenderBatchActive
#define DT_disable rlDisableDepthTest
#define DT_enable rlEnableDepthTest
#define Q_to_M QuaternionToMatrix
#define M_tran MatrixTranslate
#define M_mult MatrixMultiply
#define Q_from_A QuaternionFromAxisAngle
#define WHA WaterHeightAt
#define Q_norm QuaternionNormalize
#define Q_mul QuaternionMultiply

typedef struct object {
    const char *name;
    float mass, longDrag, cellVol, cellH, volSub;
    Vector3 size, inertia, pos, vel, angVel, force, torque, start, cells[600];
    Quaternion rot;
    int nbCells;
    Mesh mesh;
} object;

typedef struct drop { 
    Vector3 p, v; float life; } drop;

object obj[3];
drop spray[500];
Vector3 trailPos[90], trailRight[90];
float trailAge[90], trailSpeed[90], trailTimer, sprayAcc, throttle, rudder;
int trailNext;

float frand(float a, float b) { 
    return a+(b-a)*GetRandomValue(0, 1000)/1000.0f; }

// Hull shape
float hull_width(float zn) { return B/2*sqrtf(fmaxf(0, 1 - powf((zn + 0.1f) / 1.1f, 2)));}
float hull_deck(float zn) { return 0.9f+0.4f*powf(fmaxf(zn, 0), 2);}
float hull_keel(float zn) { return -0.65f+0.65f*powf(fmaxf(zn, 0), 3);}

bool inside_hull(Vector3 p){
    float zn = p.z/(L/2);
    if (zn < -1 || zn > 1){ 
        return false;
    }
    float hw = hull_width(zn), deck = hull_deck(zn), keel = hull_keel(zn), hauteurA = Lerp(keel,deck,0.45f);
    if (p.y > deck || p.y < keel){
         return false;
    }
    return fabsf(p.x) <= (p.y >= hauteurA ? Lerp(0.88f * hw, hw, (p.y - hauteurA) / (deck-hauteurA)) : 0.88f*hw*(p.y-keel)/(hauteurA-keel));
}

//mesh construction
float *pos_summit; 
float *normal;
unsigned char *colors;
int nb_triangles;

void tri(Vector3 a, Vector3 b, Vector3 c, Vector3 inside, Color col) {
    Vector3 nor = CROSS(SUB(b, a), SUB(c, a));
    if (LEN(nor) < 1e-6f){ 
        return;
    }
    nor = NORM(nor);
    if (DOT(nor, SUB(MUL(ADD(ADD(a, b), c), 1.0f / 3), inside)) < 0) {
        Vector3 tmp = b; b = c; c = tmp;
        nor = Vector3Negate(nor);
    }
    Vector3 p[3] = {a, b, c};
    for (int k = 0, i = nb_triangles * 3; k < 3; k++, i++) {
        pos_summit[3*i] = p[k].x; pos_summit[3*i+1] = p[k].y; pos_summit[3*i+2] = p[k].z;
        normal[3*i] = nor.x; normal[3*i+1] = nor.y; normal[3*i+2] = nor.z;
        colors[4*i] = col.r; colors[4*i+1] = col.g; colors[4*i+2] = col.b; colors[4*i+3] = 255;
    }
    nb_triangles++;
}

void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 in, Color col) { 
    tri(a, b, c, in, col); tri(a, c, d, in, col); 
}

void box(Vector3 c, Vector3 s, Color col) {
    Vector3 p[8];
    for (int i = 0; i < 8; i++){
        p[i] = V3(c.x + ((i & 1) ? 0.5f : -0.5f) * s.x, c.y + ((i & 2) ? 0.5f : -0.5f) * s.y, c.z + ((i & 4) ? 0.5f : -0.5f) * s.z);
    }
    int f[6][4] = {{0, 1, 3, 2}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 3, 7, 5}};
    for (int i=0; i < 6; i++){
        quad(p[f[i][0]], p[f[i][1]], p[f[i][2]], p[f[i][3]], c, col);
    }
}

void cylinder(float y, float r, float h, int seg, Color col){
    Vector3 base = V3(0, y, 0), top = V3(0, y + h, 0), mid = V3(0, y + h / 2, 0);
    for (int i = 0; i < seg; i++) {
        float a0 = 2*PI*i/seg, a1 = 2*PI * (i+1)/seg;
        Vector3 b0=V3(r * cosf(a0), y, r * sinf(a0)), b1=V3(r * cosf(a1), y, r * sinf(a1));
        Vector3 t0=V3(b0.x, y + h, b0.z), t1=V3(b1.x, y + h, b1.z);
        quad(b0, b1, t1, t0, mid, col);
        tri(top, t0, t1, mid, col);
        tri(base, b0, b1, mid, col);
    }
}

//here I used AI to correctly understand and code the different mesh functiions (maillages)

void begin_mesh(int maxTri) { 
    pos_summit = MemAlloc(maxTri*36); 
    normal = MemAlloc(maxTri*36);
    colors = MemAlloc(maxTri*12); 
    nb_triangles= 0; 
}

Mesh end_mesh(void){
    Mesh m = {0};

    m.vertexCount = nb_triangles*3;
    m.triangleCount = nb_triangles;
    m.vertices = pos_summit;
    m.normals = normal;
    m.colors = colors;

    UploadMesh(&m, false);
    return m;
}

Mesh boat_mesh(void) {
    Color white = {235, 235, 230, 255}, red = {160, 35, 35, 255}, blue = {30, 60, 120, 255}, dark = {25, 35, 45, 255};
    Color cols[6] = {blue, white, red, red, white, blue};
    Vector3 s[21][7];
    begin_mesh(3000);
    for (int i = 0; i <= 20; i++) {
        float zn = -1+i / 10.0f, z=zn*L/2, hw = hull_width(zn), deck=hull_deck(zn), keel=hull_keel(zn), hauteurA=Lerp(keel, deck, 0.45f);
        s[i][0] = V3(-hw, deck, z);
        s[i][2] = V3(-0.88f * hw, hauteurA, z);
        s[i][1] = Vector3Lerp(s[i][0], s[i][2], 0.3f);
        s[i][3] = V3(0, keel, z);
        s[i][4] = V3(0.88f * hw, hauteurA, z);
        s[i][5] = V3(-s[i][1].x, s[i][1].y, z);
        s[i][6] = V3(hw, deck, z);
    }
    for (int i= 0; i<20; i++) {
        float zm = (s[i][0].z+s[i + 1][0].z)/ 2;
        for (int k = 0; k < 6; k++){ 
            quad(s[i][k], s[i][k+1], s[i+1][k+1], s[i+1][k], V3(0, -0.1f,zm*0.9f), cols[k]);
        }
        quad(s[i][0], s[i][6], s[i+1][6], s[i+1][0], V3(0, -1, zm), (Color){ 150, 110, 70, 255 });
        for (int k =0; k<=6; k+=6) { 
            Vector3 a = s[i][k], c= s[i+ 1][k], a2 = ADD(a, V3(0, 0.25f, 0)), c2=ADD(c, V3(0, 0.25f, 0));
            quad(a, c, c2, a2, V3(0, a.y, zm), white);
            quad(a, c, c2, a2, V3(a.x * 3, a.y, zm), white);
        }
    }
    Vector3 tc = V3(0, (s[0][0].y + s[0][3].y) / 2, s[0][0].z), in = V3(0, tc.y, 0);
    for (int k = 0; k < 6; k++){ 
        tri(tc, s[0][k], s[0][k + 1], in, white); }
    quad(s[0][0], s[0][6], ADD(s[0][6], V3(0, 0.25f, 0)), ADD(s[0][0], V3(0, 0.25f, 0)), in, white);

    float deck = hull_deck(-0.2f);
    box(V3(0, deck + 0.65f, -0.9f), V3(1.9f, 1.3f, 2.6f), white);
    box(V3(0, deck + 1.36f, -0.9f), V3(2.1f, 0.12f, 2.9f), blue);
    box(V3(0, deck + 0.85f, 0.41f), V3(1.6f, 0.5f, 0.04f), dark);
    box(V3(-0.96f, deck + 0.85f, -0.9f), V3(0.04f, 0.45f, 2.0f), dark);
    box(V3(0.96f, deck + 0.85f, -0.9f), V3(0.04f, 0.45f, 2.0f), dark);
    box(V3(0, deck + 2.6f, -1.2f), V3(0.1f, 2.4f, 0.1f), (Color){ 90, 95, 100, 255 });
    box(V3(0, deck + 3.55f, -1.55f), V3(0.03f, 0.35f, 0.6f), red);
    box(V3(0, deck + 1.75f, -1.9f), V3(0.35f, 0.7f, 0.35f), dark);
    box(V3(0.97f, deck + 0.3f, -1.9f), V3(0.06f, 0.4f, 0.4f), (Color){ 230, 90, 30, 255 });
    return end_mesh();
}

Mesh buoy_mesh(void){
    Color red = { 200, 40, 40, 255 };
    begin_mesh(600);
    cylinder(-0.4f, 0.85f, 0.4f, 24, red);
    cylinder(0, 0.86f, 0.2f, 24, (Color){ 240, 240, 240, 255 });
    cylinder(0.2f, 0.85f, 0.2f, 24, red);
    cylinder(0.4f, 0.08f, 1.4f, 8, (Color){ 220, 220, 220, 255 });
    box(V3(0, 1.9f, 0), V3(0.25f, 0.25f, 0.25f), (Color){ 255, 220, 40, 255 });
    return end_mesh();
}

Mesh crate_mesh(void) {
    Color plank = { 105, 70, 40, 255 };
    begin_mesh(400);
    box(V3(0, 0, 0), V3(1.5f, 1.5f, 1.5f), (Color){ 165, 120, 70, 255 });
    for (int i = -1; i <= 1; i += 2){
        for (int j = -1; j <= 1; j += 2) {
            box(V3(0, i * 0.78f, j * 0.78f), V3(1.58f, 0.14f, 0.14f), plank);
            box(V3(i * 0.78f, 0, j * 0.78f), V3(0.14f, 1.58f, 0.14f), plank);
            box(V3(i * 0.78f, j * 0.78f, 0), V3(0.14f, 0.14f, 1.58f), plank);
        }
    }
    return end_mesh();
}

//physics
void set_inertia(object *o){
    Vector3 size = (*o).size;
    float x2 = size.x * size.x;
    float y2 = size.y * size.y;
    float z2 = size.z * size.z;

    (*o).inertia = MUL(V3(y2 + z2, x2 + z2, x2 + y2), (*o).mass / 12);
}

float volume(object *o) { 
    return (*o).nbCells * (*o).cellVol; 
}


void make_obj(object *o, int type, const char *name, Vector3 size, Vector3 start, int nx, int ny, int nz, float y0, float y1){
    *o = (object){0};
    (*o).name = name;
    (*o).size = size;
    (*o).start = start;
    (*o).rot = QID();
    (*o).longDrag = 1.0f;

    if (type == 0) {
        (*o).longDrag = 0.12f;
    }

    float width_cell = size.x / nx;
    float height_cell = (y1 - y0) / ny;
    float lenght_cell = size.z / nz;

    (*o).cellH = height_cell;
    (*o).cellVol = width_cell * height_cell*lenght_cell;
    for (int i = 0; i < nx; i++) {
        float x = -size.x / 2 + (i + 0.5f)*width_cell;
        for (int j = 0; j < ny; j++) {
            float y = y0 + (j + 0.5f)*height_cell;
            for (int k = 0; k < nz; k++) {
                float z = -size.z / 2 + (k + 0.5f)*lenght_cell;
                Vector3 centre = V3(x, y, z);
                if (type == 0 && !inside_hull(centre)) {
                    continue;
                }
                if (type == 1 && x * x + z * z > 0.85f*0.85f) {
                    continue;
                }

                (*o).cells[(*o).nbCells] = centre;
                (*o).nbCells++;
            }
        }
    }
}

void reset(object *o){
    (*o).pos = (*o).start;
    (*o).rot = QID();
    (*o).vel = (*o).angVel = (*o).force = (*o).torque = V0();
}

void forceAt(object *o, Vector3 force, Vector3 pointApp){
    (*o).force = ADD((*o).force, force);
    (*o).torque = ADD((*o).torque, CROSS(SUB(pointApp, (*o).pos), force));
}

Vector3 world(object *o, Vector3 p) { 
    return ADD((*o).pos, ROT(p, (*o).rot)); 
}

// Archimede
void buoyancy(object *o, float time) {
    Vector3 fwd = ROT(V3(0, 0, 1), (*o).rot);
    (*o).volSub = 0;
    for (int i = 0; i < (*o).nbCells; i++) {
        Vector3 c = world(o, (*o).cells[i]);
        float frac = Clamp((WHA(c.x, c.z, time) - c.y) / (*o).cellH + 0.5f, 0, 1);
        if (frac <= 0){ 
            continue;
        }
        float vol = (*o).cellVol*frac;
        (*o).volSub += vol;
        forceAt(o, V3(0, RHO*G*vol, 0), c);
        Vector3 v = ADD((*o).vel, CROSS((*o).angVel, SUB(c, (*o).pos)));
        float vf = DOT(v, fwd);
        forceAt(o, MUL(ADD(SUB(v, MUL(fwd, vf)), MUL(fwd, vf*(*o).longDrag)), -3000*vol), c);
    }
    (*o).force = ADD((*o).force, MUL((*o).vel, -0.02f * (*o).mass));
    (*o).torque = ADD((*o).torque, MUL((*o).angVel, -0.02f * (*o).mass));
}

void steer(object *o, float dt, float time){
    float want_T = IsKeyDown(KEY_DOWN) ? -0.5f : IsKeyDown(KEY_UP) ? 1 : 0;
    float want_R = IsKeyDown(KEY_RIGHT) ? -1 : IsKeyDown(KEY_LEFT) ? 1 : 0;
    throttle += (want_T - throttle)*Clamp(dt*1.5f, 0, 1);
    rudder += (want_R - rudder)*Clamp(dt*3, 0, 1);
    Vector3 fwd = ROT(V3(0, 0, 1), (*o).rot), prop = world(o, V3(0, -0.35f, -L / 2 + 0.4f));
    if (WHA(prop.x, prop.z, time) > prop.y) { 
        forceAt(o, MUL(fwd, throttle * 26000), prop);
        (*o).torque = ADD((*o).torque, MUL(ROT(V3(0, 1, 0), (*o).rot), rudder*(7000*DOT((*o).vel, fwd) + 5000)));
    }
}

// Euler
void integrate(object *o, float dt) {
    (*o).force.y -= (*o).mass * G;
    (*o).vel = ADD((*o).vel, MUL((*o).force, dt / (*o).mass));
    (*o).pos = ADD((*o).pos, MUL((*o).vel, dt));
    Vector3 tl = ROT((*o).torque, QuaternionInvert((*o).rot));
    (*o).angVel = ADD((*o).angVel, MUL(ROT(V3(tl.x / (*o).inertia.x, tl.y / (*o).inertia.y, tl.z / (*o).inertia.z), (*o).rot), dt));
    float w = LEN((*o).angVel);
    if (w > 15) { (*o).angVel = MUL((*o).angVel, 15 / w); w = 15; }
    if (w > 1e-6f) (*o).rot = Q_norm(Q_mul(Q_from_A(MUL((*o).angVel, 1 / w), w * dt), (*o).rot));
    if (LEN((*o).vel) > 40) (*o).vel = MUL(NORM((*o).vel), 40);
    (*o).force = (*o).torque = V0();
}

// Bow and wake
void effects(object *o, float dt, float time) {
    Vector3 fwd = ROT(V3(0, 0, 1), (*o).rot), right = ROT(V3(-1, 0, 0), (*o).rot), bow = world(o, V3(0, 0.35f, L/2-1.5f));
    float speed = LEN(V3((*o).vel.x, 0, (*o).vel.z));
    if (WHA(bow.x, bow.z, time) - bow.y > -0.6f) sprayAcc += (fmaxf(0, speed - 1) * 18 + fmaxf(0, -(*o).vel.y-1)*80)*dt;
    for (; sprayAcc > 1; sprayAcc--){
        for (int i = 0; i < 500; i++) {
            if (spray[i].life > 0){ 
                continue;
            }
            float side = GetRandomValue(0, 1) ? 1 : -1;
            Vector3 p = ADD(bow, MUL(right, side * frand(0.6f, 1.1f)));
            p.y = WHA(p.x, p.z, time) + 0.1f;
            Vector3 v = ADD(MUL((*o).vel, 0.5f), MUL(right, side * frand(1.0f, 2.5f) * (0.5f + speed * 0.15f)));
            v.y = frand(1.0f, 2.5f) + speed * 0.25f;
            v = ADD(v, MUL(fwd, frand(-0.5f, 0.5f)));
            spray[i] = (drop){ p, v, frand(0.6f, 1.2f) };
            break;
        }
    }
    for (int i = 0; i < 500; i++) {
        drop *d = &spray[i];
        if (d->life <= 0){ 
            continue;
        }
        d->v.y -= G * dt;
        d->p = ADD(d->p, MUL(d->v, dt));
        d->life -= dt;
        if (d->p.y < WHA(d->p.x, d->p.z, time) - 0.2f){ 
            d->life = 0;
        }
    }
    for (int i = 0; i < 90; i++) trailAge[i] += dt;
    if ((trailTimer += dt) > 0.08f) {
        trailTimer = 0;
        trailPos[trailNext] = world(o, V3(0, 0, -L/2));
        trailRight[trailNext] = right;
        trailAge[trailNext] = 0;
        trailSpeed[trailNext] = speed;
        trailNext = (trailNext + 1) % 90;
    }
}

//functions used by Mahe
int WakeCount(void) { 
    return 270; }

Vector3 WakePoint(int i, float *s){
    int k = i / 3; 
    int side = i % 3 - 1;
    float a = trailAge[k], sp = trailSpeed[k];
    *s = a > 7 ? 0 : Clamp(1 - a / 6, 0, 1)*Clamp((sp - 1) / 5, 0, 1)*(side ? 0.85f : 0.3f);
    return ADD(trailPos[k], MUL(trailRight[k], side * (0.9f + a * sp * 0.12f)));
}

void Obj_init(void) {
    make_obj(&obj[0], 0, "Boat", V3(B, 2, L), V3(0, 0, 0), 6, 5, 16, -0.65f, 1.3f);
    make_obj(&obj[1], 1, "Buoy", V3(1.7f, 0.8f, 1.7f), V3(-10, 0.4f, 7), 4, 2, 4, -0.4f, 0.4f);
    make_obj(&obj[2], 2, "Crate", V3(1.5f, 1.5f, 1.5f), V3(9, 0.4f, 6), 3, 3, 3, -0.75f, 0.75f);
    obj[0].mass = 0.42f*volume(&obj[0])*RHO;
    obj[1].mass = 300;
    obj[2].mass = 2000;
    obj[0].mesh = boat_mesh();
    obj[1].mesh = buoy_mesh();
    obj[2].mesh = crate_mesh();
    for (int i = 0; i < 3; i++) { set_inertia(&obj[i]); reset(&obj[i]); }
    for (int i = 0; i < 90; i++) trailAge[i] = 100;
}

void Obj_step(float dt, float t) {
    steer(&obj[0], dt, t);
    for (int i = 0; i < 3; i++) {
        buoyancy(&obj[i], t);
        integrate(&obj[i], dt);
        if (obj[i].pos.y < -25) reset(&obj[i]);
    }
    effects(&obj[0], dt, t);
}

void Obj_reset(void) {
    for (int i = 0; i < 3; i++) reset(&obj[i]);
    for (int i = 0; i < 90; i++) trailAge[i] = 100;
    for (int i = 0; i < 500; i++) spray[i].life = 0;
    throttle = rudder = 0;
}

int Obj_count(void) { 
    return 3; }

const char *Obj_name(int i) { 
    return obj[i].name; 
}

float Obj_submerged_percent(int i) { return 100*obj[i].volSub/volume(&obj[i]); }
float Obj_density(int i) { return obj[i].mass / volume(&obj[i]); }
float Crate_get_mass(void) { return obj[2].mass; }

void Crate_set_mass(float m) { 
    obj[2].mass = fmaxf(m, 1); set_inertia(&obj[2]); 
}

Vector3 BoatPosition(void) { 
    return obj[0].pos; 
}

float Boat_speed(void) { 
    return LEN(V3(obj[0].vel.x, 0, obj[0].vel.z)); 
}

void Obj_draw(Material *mat, bool cells, float time) {
    mat->maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    for (int n = 0; n < 3; n++) {
        object *o = &obj[n];
        if (!cells) {
            DrawMesh((*o).mesh, *mat, M_mult(Q_to_M((*o).rot), M_tran((*o).pos.x, (*o).pos.y, (*o).pos.z)));
            continue;
        }
        DRB_act();
        DT_disable(); 
        for (int i = 0; i < (*o).nbCells; i++) {
            Vector3 c = world(o, (*o).cells[i]);
            DrawCube(c, 0.2f, 0.2f, 0.2f, WHA(c.x, c.z, time) > c.y ? SKYBLUE : WHITE);
        }
        DRB_act();
        DT_enable();
    }
    for (int i = 0; i < 500; i++)
        if (spray[i].life > 0) DrawCube(spray[i].p, 0.16f, 0.16f, 0.16f, Fade(RAYWHITE, Clamp(spray[i].life*1.5f, 0, 1)));
}







