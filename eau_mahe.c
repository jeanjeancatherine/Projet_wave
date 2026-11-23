#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdio.h>

void Obj_init(void);
void Obj_step(float dt, float t);
void Obj_draw(Material *mat, bool cells, float t);
void Obj_reset(void);
int Obj_count(void);
const char *Obj_name(int i);
float Obj_submerged_percent(int i);
float Obj_density(int i);
float Crate_get_mass(void);
void Crate_set_mass(float m);
Vector3 BoatPosition(void);
float Boat_speed(void);
int WakeCount(void);
Vector3 WakePoint(int i, float *strength);

#define SIZE 160.0f
#define RES 200
#define N (RES + 1)
#define STEP (SIZE / RES)
#define V_norm Vector3Normalize

// Perlin Noise
int perm[512];

void initPerm(unsigned int s){
    int p[256];
    for (int i = 0; i < 256; i++){
         p[i] = i;}
    s = s * 2654435761u + 1u;
    for (int i = 255; i > 0; i--) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        int j = s % (i + 1), tmp = p[i];
        p[i] = p[j]; p[j] = tmp;
    }
    for (int i = 0; i < 512; i++){
         perm[i] = p[i & 255];
    }
}

float fade(float t) { 
    return t*t*t*(t*(t * 6 - 15)+10);}
float lerp(float t, float a, float b) { 
    return a+t * (b-a);}

float grad(int h, float x, float y, float z){
    h &= 15;
    float u =h < 8 ? x : y, v=h < 4 ? y : (h == 12 || h == 14 ? x : z);
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

float perlin(float x, float y, float z) {
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    int Z = (int)floorf(z) & 255;
    x -= floorf(x);
    y -= floorf(y);
    z -= floorf(z);

    float u = fade(x);
    float v = fade(y);
    float w = fade(z);
    int A = perm[X] + Y;
    int B = perm[X + 1] + Y;
    int AA = perm[A] + Z;
    int AB = perm[A + 1] + Z;
    int BA = perm[B] + Z;
    int BB = perm[B + 1] + Z;

    float x00 = lerp(u, grad(perm[AA], x, y, z),
                        grad(perm[BA], x - 1, y, z));
    float x10 = lerp(u, grad(perm[AB], x, y - 1, z),
                        grad(perm[BB], x - 1, y - 1, z));
    float x01 = lerp(u, grad(perm[AA + 1], x, y, z - 1),
                        grad(perm[BA + 1], x - 1, y, z - 1));
    float x11 = lerp(u, grad(perm[AB + 1], x, y - 1, z - 1),
                        grad(perm[BB + 1], x - 1, y - 1, z - 1));
    float faceZ0 = lerp(v, x00, x10);
    float faceZ1 = lerp(v, x01, x11);

    return lerp(w, faceZ0, faceZ1);
}

// Waves
float weather = 0.35f, windAngle = 30, swellAmp, swellLen, detailAmp, detailLen, detailSpeed, persistence, evolution;
int octaves = 4;
float swDx[3], swDz[3], swK[3], swW[3], swA[3], wc, ws;

void preparewaves(void) {
    float ang[3] = { 0, 25, -20 }, len[3] = {1, 0.62f, 0.41f}, amp[3] = {1, 0.45f, 0.25f};
    for (int i = 0; i < 3; i++) {
        float a = (windAngle + ang[i]) * DEG2RAD;
        swDx[i] = cosf(a); swDz[i] = sinf(a);
        swK[i] = 2 * PI / (swellLen * len[i]);
        swW[i] = sqrtf(9.81f * swK[i]);
        swA[i] = swellAmp * amp[i];
    }
    wc = cosf(windAngle*DEG2RAD); ws = sinf(windAngle*DEG2RAD);
}


void setweather(float w) {
    weather = Clamp(w, 0, 1);
    swellAmp = 1.5f*weather;          
    detailAmp = 0.9f*weather;
    swellLen = Lerp(18, 55, weather);   
    detailLen = Lerp(7, 13, weather);
    detailSpeed = Lerp(0.8f, 4, weather);
    persistence = Lerp(0.45f, 0.55f, weather);
    evolution = Lerp(0.15f, 0.4f, weather);
    preparewaves();
}

// Water Height, used by Jean too
float WaterHeightAt(float x, float z, float t) {
    float h=0;
    for (int i = 0; i < 3; i++) {
        float s=(sinf(swK[i]*(x * swDx[i]+ z * swDz[i])-swW[i] * t + i*1.7f) + 1)*0.5f;
        h += swA[i]*(2*s*s-0.75f);
    }
    float u = x*wc + z*ws - detailSpeed*t, v = (-x*ws + z*wc)/1.8f;
    float freq = 1 / detailLen, amp = 1, sum = 0, tot =0;
    for (int o = 0; o < octaves; o++) {
        sum += amp * perlin(u*freq, t* evolution +o*31.7f, v*freq);
        tot += amp; amp *= persistence; freq *= 2;
    }
    return h + detailAmp * sum / tot * 2;
}

// Water grid
Mesh water;
float H[N * N], foamTmp[N * N], ox, oz;
bool goodNormals = true;
void createWater(void){
    water = (Mesh){ .vertexCount = N*N, .triangleCount = RES*RES*2};
    water.vertices = MemAlloc(N*N*12);
    water.normals = MemAlloc(N*N*12);
    water.texcoords = MemAlloc(N*N*8);
    water.indices = MemAlloc(RES*RES*12);
    int k = 0;
    for (int z = 0; z < RES; z++){
        for (int x = 0; x < RES; x++) {
            int i = z * N + x, q[6] = { i, i + N, i + 1, i + 1, i + N, i + N + 1 };
            for (int j = 0; j < 6; j++) water.indices[k++] = q[j];
        }
    }
    UploadMesh(&water, true);
}
#define AT(x, z) H[(z) * N + (x)]

void updateWater(float t, Vector3 c){
    float *v=water.vertices, *n=water.normals, *f=water.texcoords;
    ox = roundf(c.x / STEP)*STEP;
    oz = roundf(c.z / STEP) * STEP;
    for (int i = 0; i < N * N; i++) {
        v[3 * i] = ox - SIZE / 2 + i % N * STEP;
        v[3 * i + 2] = oz - SIZE / 2 + i / N * STEP;
        v[3 * i + 1] = H[i] = WaterHeightAt(v[3 * i], v[3 * i + 2], t);
    }

    float hMax = swellAmp * 2.1f + detailAmp;
    for (int z = 0; z < N; z++){
        for (int x=0; x < N; x++) {
            int i = z*N+x; 
            int x0 = x ? x-1: x; 
            int x1 = x < N-1 ? x+1: x;
            int z0 = z ? z-1: z; 
            int z1 = z < N-1 ? z+1: z;
            Vector3 nor = {0, 1, 0};
            if (goodNormals) {
                float penteX = (AT(x1, z) - AT(x0, z))/((x1 - x0)*STEP);
                float penteZ = (AT(x, z1) - AT(x, z0))/((z1 - z0)*STEP);

                nor = V_norm((Vector3){-penteX, 1, -penteZ});
            }
            n[3*i] = nor.x; 
            n[3*i+1] =nor.y;
            n[3*i+2] = nor.z;

            // ecume
            float lap = (AT(x1, z) + AT(x0, z) + AT(x, z1) + AT(x, z0) - 4 * H[i]) /(STEP*STEP);
            float crest = Clamp((-lap - 0.05f)*4, 0, 1) *Clamp((H[i]/ (hMax +0.01f) - 0.15f)*2.5f, 0, 1);
            if (crest > 0){ 
                crest *= Clamp(perlin(v[3*i] *0.35f, t*0.5f, v[3*i+2]* 0.35f)*1.5f+0.6f, 0, 1);
            }
            foamTmp[i] = Clamp(crest * (0.3f + weather), 0, 1);
        }
    }
    for (int k = 0; k < WakeCount(); k++) {
        float s;
        Vector3 p = WakePoint(k, &s);
        if (s <= 0.01f){
             continue;
        }
        int gx = roundf((p.x-ox+SIZE/2)/STEP);
        int gz = roundf((p.z-oz+SIZE/2)/STEP);
        for (int dz = -2; dz <= 2; dz++){
            for (int dx = -2; dx <= 2; dx++) {
                int x = gx + dx, z = gz + dz;
                if (x >= 0 && x < N && z >= 0 && z < N) {
                    int i = z * N + x;
                    float distance = sqrtf(dx*dx+dz*dz);
                    float foam = s * (1 - distance/2.5f);
                    if (foam > foamTmp[i]) {
                        foamTmp[i] = foam;
                    }
                }
            }  
        }     
    }
    for (int z = 0; z < N; z++) {
        for (int x = 0; x < N; x++) {
            int i = z*N+x;
            float centre = foamTmp[i];

            float sum = 4*centre;
            sum += (x > 0)     ? foamTmp[i-1] : centre;
            sum += (x < N - 1) ? foamTmp[i+1] : centre;
            sum += (z > 0)     ? foamTmp[i-N] : centre;
            sum += (z < N - 1) ? foamTmp[i+N] : centre;

            f[2*i] = sum/8;
        }
    }
    UpdateMeshBuffer(water, 0, v, N * N * 12, 0);
    UpdateMeshBuffer(water, 1, f, N * N * 8, 0);
    UpdateMeshBuffer(water, 2, n, N * N * 12, 0);
}

void drawNormals(void){
    float *v = water.vertices, *n = water.normals;
    for (int z = N / 4; z < 3*N / 4; z += 4){
        for (int x = N/4; x < 3*N/4; x += 4) {
            float *a = v+3 * (z*N+x), *b = n+3 * (z*N+x);
            DrawLine3D((Vector3){ a[0], a[1], a[2] }, (Vector3){ a[0] + b[0], a[1] + b[1], a[2] + b[2] }, YELLOW);
        }
    }
}

//shader
const char *vs=
    "#version 330\n"
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec2 vertexTexCoord; in vec4 vertexColor;\n"
    "uniform mat4 mvp, matModel, matNormal;\n"
    "out vec3 pos, nor, vcol; out float foamV;\n"
    "void main() {\n"
    "  pos = vec3(matModel * vec4(vertexPosition, 1.0));\n"
    "  nor = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));\n"
    "  foamV = vertexTexCoord.x; vcol = vertexColor.rgb;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

const char *fs =
    "#version 330\n"
    "in vec3 pos, nor, vcol; in float foamV; out vec4 finalColor;\n"
    "uniform vec4 colDiffuse; uniform vec3 lightDir, viewPos, center, horizon;\n"
    "uniform int isWater, foam; uniform float hMax, fogStart, fogEnd;\n"
    "void main() {\n"
    "  vec3 n = normalize(nor), L = normalize(-lightDir), V = normalize(viewPos - pos), H = normalize(L + V), col;\n"
    "  float diff = max(dot(n, L), 0.0);\n"
    "  if (isWater == 1) {\n"
    "    float h = clamp(pos.y / hMax * 0.5 + 0.5, 0.0, 1.0);\n"
    "    col = mix(vec3(0.01, 0.09, 0.16), vec3(0.03, 0.30, 0.38), h * h) * (0.55 + 0.45 * diff);\n"
    "    col += vec3(0.02, 0.22, 0.20) * pow(max(dot(V, -L), 0.0), 3.0) * h;\n"
    "    vec3 sky = mix(horizon, vec3(0.30, 0.52, 0.78), clamp(reflect(-V, n).y * 2.0, 0.0, 1.0));\n"
    "    col = mix(col, sky, (0.02 + 0.98 * pow(1.0 - max(dot(n, V), 0.0), 5.0)) * 0.8);\n"
    "    col += vec3(1.0, 0.95, 0.85) * pow(max(dot(n, H), 0.0), 600.0) * 2.0;\n"
    "    if (foam == 1) col = mix(col, vec3(0.92, 0.95, 0.97) * (0.75 + 0.25 * diff), foamV);\n"
    "  } else col = colDiffuse.rgb * vcol * (0.30 + 0.70 * diff) + vec3(0.12) * pow(max(dot(n, H), 0.0), 32.0);\n"
    "  finalColor = vec4(mix(col, horizon, smoothstep(fogStart, fogEnd, length(pos.xz - center.xz))), 1.0);\n"
    "}\n";


//interface
int dragging = -1;
float slider(int id, float x, float y, float w, const char *label, const char *val, float value, float min, float max) {
    Vector2 m = GetMousePosition();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(m, (Rectangle){ x - 4, y + 12, w + 8, 22 })){
        dragging = id;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)){ 
        dragging = -1;
    }
    if (dragging == id){ 
        value = min + Clamp((m.x - x) / w, 0, 1) * (max - min);
    }
    float k = (value-min) / (max-min);
    DrawText(label, x, y, 14, RAYWHITE);
    DrawText(val, x + w - MeasureText(val, 14), y, 14, LIGHTGRAY);
    DrawRectangle(x, y + 18, w, 10, (Color){ 60, 70, 85, 255 });
    DrawRectangle(x, y + 18, w * k, 10, (Color){ 70, 150, 220, 255 });
    DrawCircle(x + w * k, y + 23, 8, RAYWHITE);
    return value;
}

//To have a good main structure, I used AI to know exactly how to combine all the functions that I created

int main(void) {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1280, 720, "Water Simulation");
    SetTargetFPS(60);
    initPerm(0);
    setweather(weather);
    createWater();

    Shader sh = LoadShaderFromMemory(vs, fs);
    const char *names[] = { "lightDir", "viewPos", "center", "horizon", "isWater", "foam", "hMax", "fogStart", "fogEnd" };
    int loc[9];
    for (int i = 0; i < 9; i++) loc[i] = GetShaderLocation(sh, names[i]);
    Vector3 horizon = { 0.72f, 0.80f, 0.86f };
    float fog[2] = { SIZE * 0.25f, SIZE * 0.48f };
    SetShaderValue(sh, loc[3], &horizon, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[7], &fog[0], SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, loc[8], &fog[1], SHADER_UNIFORM_FLOAT);
    Material mat = LoadMaterialDefault();
    mat.shader = sh;

    Obj_init();
    float crateMass = Crate_get_mass(), yaw = 0.9f, pitch = 0.32f, dist = 30, t = 0, acc = 0, lightAngle = 2.2f, dt = 1.0f / 120;
    Camera3D cam = { .up = { 0, 1, 0 }, .fovy = 50, .projection = CAMERA_PERSPECTIVE };
    Vector3 target = { 0 };
    bool showNormals = 0, turnLight = 0, wire = 0, cells = 0, pause = 0, foam = 1, follow = 1;
    struct { int key; bool *on; } keys[] = { { KEY_N, &goodNormals }, { KEY_D, &showNormals }, { KEY_L, &turnLight }, { KEY_W, &wire },
                                             { KEY_C, &cells }, { KEY_F, &foam }, { KEY_V, &follow }, { KEY_SPACE, &pause } };
    Rectangle panel = {10, 10, 330, 175};
    Color dark = Fade((Color){20, 28, 40, 255}, 0.85f);

    while (!WindowShouldClose()) {
        for (int i = 0; i < 8; i++) if (IsKeyPressed(keys[i].key)) *keys[i].on = !*keys[i].on;
        if (IsKeyPressed(KEY_R)) Obj_reset();

        if (!CheckCollisionPointRec(GetMousePosition(), panel) && dragging == -1) {
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                yaw -= GetMouseDelta().x * 0.005f;
                pitch = Clamp(pitch + GetMouseDelta().y * 0.005f, 0.05f, 1.45f);
            }
            dist = Clamp(dist-GetMouseWheelMove()*2, 8, 70);
        }

        float frame = fminf(GetFrameTime(), 0.1f);
        if (!pause)
            for (acc += frame; acc >= dt; acc -= dt) Obj_step(dt, t += dt);
        if (turnLight) lightAngle += frame * 0.5f;

        Vector3 goal = follow ? BoatPosition() : (Vector3){ 0 };
        goal.y = 0;
        target = Vector3Lerp(target, goal, Clamp(frame * 3, 0, 1));
        cam.target = (Vector3){ target.x, 1, target.z };
        cam.position = (Vector3){ target.x + dist * cosf(pitch) * sinf(yaw), 1 + dist * sinf(pitch), target.z + dist * cosf(pitch) * cosf(yaw) };

        double t0 = GetTime();

        updateWater(t, target);
        
        float waterMs = (GetTime() - t0) * 1000;

        Vector3 light = Vector3Normalize((Vector3){ cosf(lightAngle), -0.55f, sinf(lightAngle) }), c = { ox, 0, oz };
        float hMax = swellAmp * 2.1f + detailAmp + 0.05f;
        int one = 1, zero = 0, foamOn = foam;
        SetShaderValue(sh, loc[0], &light, SHADER_UNIFORM_VEC3);
        SetShaderValue(sh, loc[1], &cam.position, SHADER_UNIFORM_VEC3);
        SetShaderValue(sh, loc[2], &c, SHADER_UNIFORM_VEC3);
        SetShaderValue(sh, loc[5], &foamOn, SHADER_UNIFORM_INT);
        SetShaderValue(sh, loc[6], &hMax, SHADER_UNIFORM_FLOAT);

        Vector3 far = Vector3Add(cam.position, Vector3Scale(Vector3Normalize((Vector3){ cam.target.x - cam.position.x, 0, cam.target.z - cam.position.z }), 5000));
        far.y = 0;

        BeginDrawing();
        ClearBackground((Color){ 184, 204, 219, 255 });
        DrawRectangleGradientV(0, 0, GetScreenWidth(), GetWorldToScreen(far, cam).y, (Color){ 92, 140, 196, 255 }, (Color){ 184, 204, 219, 255 });

        BeginMode3D(cam);
        SetShaderValue(sh, loc[4], &one, SHADER_UNIFORM_INT);
        if (wire) rlEnableWireMode();
        DrawMesh(water, mat, MatrixIdentity());
        if (wire) rlDisableWireMode();
        SetShaderValue(sh, loc[4], &zero, SHADER_UNIFORM_INT);
        Obj_draw(&mat, cells, t);
        if (showNormals) drawNormals();
        EndMode3D();

        DrawRectangleRec(panel, dark);
        char txt[32];
        sprintf(txt, "%s", weather < 0.3f ? "calm" : weather < 0.7f ? "agitated" : "storm");
        setweather  (slider(1, 22, 22, 306, "Weather", txt, weather, 0, 1));
        sprintf(txt, "%.0f deg", windAngle);
        windAngle = slider(2, 22, 62, 306, "Wind Direction", txt, windAngle, 0, 360);
        preparewaves();
        sprintf(txt, "%d", octaves);
        octaves = slider(3, 22, 102, 306, "Octaves (Perlin Noise)", txt, octaves, 1, 6) + 0.5f;
        sprintf(txt, "%.0f kg", crateMass);
        crateMass = slider(4, 22, 142, 306, "Crate Mass", txt, crateMass, 500, 5000);
        if (fabsf(crateMass - Crate_get_mass()) > 1) Crate_set_mass(crateMass);

        DrawRectangle(0, GetScreenHeight() - 28, GetScreenWidth(), 28, dark);
        DrawText("Arrow keys : steer the boat | Right click : camera | V follow the boat | N normals | D show normals | F foam | L light | W wireframe | C cells | Space pause | R reset",
                 10, GetScreenHeight() - 20, 13, RAYWHITE);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
