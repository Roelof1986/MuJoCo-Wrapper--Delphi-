// mujoco_delphi_wrapper.cpp — Delphi-compatible MuJoCo + GLFW DLL

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
#  define MJW_API extern "C" __declspec(dllexport)
#else
#  define MJW_API extern "C"
#endif

// -----------------------------------------------------------------------------
// Thread-local error text
// -----------------------------------------------------------------------------
static thread_local char g_lastErr[1024]{};
static inline void set_err(const char* s) {
    if (!s) g_lastErr[0] = 0;
    else std::snprintf(g_lastErr, sizeof(g_lastErr), "%s", s);
}

// -----------------------------------------------------------------------------
// Context struct
// -----------------------------------------------------------------------------
struct MJCtx {
    GLFWwindow*  win{nullptr};
    mjModel*     m{nullptr};
    mjData*      d{nullptr};
    mjvCamera    cam{};
    mjvOption    opt{};
    mjvScene     scn{};
    mjrContext   con{};

    // === PATCH 1: mouse-camera state ===
    bool   mouseCamEnabled{false};
    bool   btnL{false}, btnR{false}, btnM{false};
    double lastX{0}, lastY{0};
    double rotScale{0.003};   // orbit: radians per pixel
    double panScale{0.002};   // pan: world units per pixel
    double zoomScale{0.001};  // zoom: per wheel tick
};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static bool make_gl_context(GLFWwindow** out) {
    if (!glfwInit()) { set_err("GLFW init failed"); return false; }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);

    GLFWwindow* win = glfwCreateWindow(1200, 900, "MuJoCo — DLL Wrapper", nullptr, nullptr);
    if (!win) { set_err("glfwCreateWindow failed"); return false; }

    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    // zichtbaar & vooraan
    glfwShowWindow(win);
    glfwSetWindowPos(win, 80, 80);
    glfwFocusWindow(win);

    *out = win;
    return true;
}

static void destroy_ctx(MJCtx* c) {
    if (!c) return;
    if (c->m) {
        mjv_freeScene(&c->scn);
        mjr_freeContext(&c->con);
        mj_deleteData(c->d);
        mj_deleteModel(c->m);
    }
    if (c->win) {
        // callbacks loskoppelen voor de zekerheid
        glfwSetCursorPosCallback   (c->win, nullptr);
        glfwSetMouseButtonCallback (c->win, nullptr);
        glfwSetScrollCallback      (c->win, nullptr);

        glfwDestroyWindow(c->win);
        glfwTerminate();
    }
    delete c;
}

// === PATCH 2: GLFW window → MJCtx* helper (buiten extern "C") ===
static inline MJCtx* ctxFromWindow(GLFWwindow* w) {
    return static_cast<MJCtx*>(glfwGetWindowUserPointer(w));
}

// === PATCH 3: GLFW mouse callbacks (buiten extern "C") ===
static void s_cursorPosCB(GLFWwindow* w, double x, double y) {
    MJCtx* c = ctxFromWindow(w);
    if (!c || !c->mouseCamEnabled) return;

    double dx = x - c->lastX;
    double dy = y - c->lastY;
    c->lastX = x; c->lastY = y;

    if (c->btnL) {
        // orbit
        mjv_moveCamera(c->m, mjMOUSE_ROTATE_H, dx * c->rotScale, 0,              &c->scn, &c->cam);
        mjv_moveCamera(c->m, mjMOUSE_ROTATE_V, 0,              dy * c->rotScale, &c->scn, &c->cam);
    }
    if (c->btnR) {
        // pan
        mjv_moveCamera(c->m, mjMOUSE_MOVE_H, dx * c->panScale, 0,               &c->scn, &c->cam);
        mjv_moveCamera(c->m, mjMOUSE_MOVE_V, 0,              dy * c->panScale,  &c->scn, &c->cam);
    }
}

static void s_mouseBtnCB(GLFWwindow* w, int button, int action, int /*mods*/) {
    MJCtx* c = ctxFromWindow(w);
    if (!c || !c->mouseCamEnabled) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT)   c->btnL = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_RIGHT)  c->btnR = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) c->btnM = (action == GLFW_PRESS);

    double x, y;
    glfwGetCursorPos(w, &x, &y);
    c->lastX = x; c->lastY = y;
}

static void s_scrollCB(GLFWwindow* w, double /*xoff*/, double yoff) {
    MJCtx* c = ctxFromWindow(w);
    if (!c || !c->mouseCamEnabled) return;
    // positieve yoff = wiel naar voren → dichterbij (pas evt. teken aan)
    mjv_moveCamera(c->m, mjMOUSE_ZOOM, 0, -yoff * c->zoomScale, &c->scn, &c->cam);
}

// -----------------------------------------------------------------------------
// Exports
// -----------------------------------------------------------------------------
MJW_API const char* __cdecl MJW_GetLastError() { return g_lastErr; }

MJW_API void* __cdecl MJW_Create(const char* modelXML, int, int) {
    set_err(nullptr);
    if (!modelXML) { set_err("Model path null"); return nullptr; }

    GLFWwindow* win = nullptr;
    if (!make_gl_context(&win)) return nullptr;

    char error[1000] = "Could not load model";
    mjModel* m = mj_loadXML(modelXML, nullptr, error, sizeof(error));
    if (!m) { set_err(error); glfwDestroyWindow(win); glfwTerminate(); return nullptr; }

    mjData* d = mj_makeData(m);
    if (!d) { set_err("mj_makeData failed"); mj_deleteModel(m); glfwDestroyWindow(win); glfwTerminate(); return nullptr; }

    auto* ctx = new MJCtx();
    ctx->win = win; ctx->m = m; ctx->d = d;
    mjv_defaultCamera(&ctx->cam);
    ctx->cam.type = mjCAMERA_FREE;
    ctx->cam.distance = 5.0;
    mjv_defaultOption(&ctx->opt);
    mjv_defaultScene(&ctx->scn);
    mjr_defaultContext(&ctx->con);
    mjv_makeScene(m, &ctx->scn, 2000);
    mjr_makeContext(m, &ctx->con, mjFONTSCALE_150);

    // === PATCH 5: koppel user pointer + defaults voor muis-cam ===
    glfwSetWindowUserPointer(win, ctx);
    ctx->mouseCamEnabled = false;  // Delphi schakelt aan via MJW_EnableMouseCam
    ctx->rotScale  = 0.003;
    ctx->panScale  = 0.002;
    ctx->zoomScale = 0.001;

    return ctx;
}

MJW_API void __cdecl MJW_Destroy(void* h) { destroy_ctx(static_cast<MJCtx*>(h)); }

MJW_API void __cdecl MJW_Step(void* h) { auto* c = (MJCtx*)h; if (c) mj_step(c->m, c->d); }

MJW_API void __cdecl MJW_Render(void* h) {
    auto* c = (MJCtx*)h; if (!c) return;
    mjv_updateScene(c->m, c->d, &c->opt, nullptr, &c->cam, mjCAT_ALL, &c->scn);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(c->win, &viewport.width, &viewport.height);
    mjr_render(viewport, &c->scn, &c->con);
    glfwSwapBuffers(c->win);
}

MJW_API void __cdecl MJW_PollEvents(void* ) { glfwPollEvents(); }

MJW_API int  __cdecl MJW_ShouldClose(void* h) {
    auto* c = (MJCtx*)h;
    return (!c || glfwWindowShouldClose(c->win)) ? 1 : 0;
}

MJW_API void __cdecl MJW_SetTimestep(void* h, double dt) {
    auto* c = (MJCtx*)h; if (c) c->m->opt.timestep = dt;
}

// -------

// ==== helpers ====
static inline MJCtx* C(void* h) { return static_cast<MJCtx*>(h); }

// ==== simulatie / state ====
MJW_API void __cdecl MJW_Reset(void* h)
{
    auto* c = C(h); if (!c) return;
    mj_resetData(c->m, c->d);
}

// - -> GetSimTime rem. (redeclared)



// -

// --- GetSizes rem.

MJW_API void __cdecl MJW_SetCtrl(void* h, const double* ctrl, int len)
{
    auto* c = C(h); if (!c || !ctrl || len<=0) return;
    int n = (c->m->nu < len) ? (int)c->m->nu : len;
    std::memcpy(c->d->ctrl, ctrl, n * sizeof(double));
}

MJW_API int __cdecl MJW_GetQpos(void* h, double* outBuf, int len)
{
    auto* c = C(h); if (!c || !outBuf || len<=0) return 0;
    int n = (c->m->nq < len) ? (int)c->m->nq : len;
    std::memcpy(outBuf, c->d->qpos, n * sizeof(double));
    return n;
}

MJW_API int __cdecl MJW_GetQvel(void* h, double* outBuf, int len)
{
    auto* c = C(h); if (!c || !outBuf || len<=0) return 0;
    int n = (c->m->nv < len) ? (int)c->m->nv : len;
    std::memcpy(outBuf, c->d->qvel, n * sizeof(double));
    return n;
}

// ==== camera / window ====
MJW_API void __cdecl MJW_SetCameraFree(void* h, double dist, double azim, double elev,
                                       double lookX, double lookY, double lookZ)
{
    auto* c = C(h); if (!c) return;
    c->cam.type = mjCAMERA_FREE;
    c->cam.distance  = (mjtNum)dist;
    c->cam.azimuth   = (mjtNum)azim;
    c->cam.elevation = (mjtNum)elev;
    c->cam.lookat[0] = (mjtNum)lookX;
    c->cam.lookat[1] = (mjtNum)lookY;
    c->cam.lookat[2] = (mjtNum)lookZ;
}

MJW_API void __cdecl MJW_ShowAndPos(void* h, int x, int y)
{
    auto* c = C(h); if (!c) return;
    glfwShowWindow(c->win);
    glfwSetWindowPos(c->win, x, y);
    glfwFocusWindow(c->win);
}

// ==== basic info (debug / diagnose) ====
MJW_API int __cdecl MJW_GetNU(void* h)  { auto* c=C(h); return c? (int)c->m->nu : 0; }
MJW_API int __cdecl MJW_GetNQ(void* h)  { auto* c=C(h); return c? (int)c->m->nq : 0; }
MJW_API int __cdecl MJW_GetNV(void* h)  { auto* c=C(h); return c? (int)c->m->nv : 0; }

// (optioneel) kopieer heel d->ctrl terug om te loggen/inspecteren
MJW_API int __cdecl MJW_GetCtrl(void* h, double* outBuf, int len)
{
    auto* c=C(h); if(!c||!outBuf||len<=0) return 0;
    int n = (c->m->nu < len) ? (int)c->m->nu : len;
    std::memcpy(outBuf, c->d->ctrl, n * sizeof(double));
    return n;
}

// --------

// ============ ACTUATOR & JOINT FORCES ==========================

// actuator output force in actuator space (nu)
MJW_API int __cdecl MJW_GetActuatorForce(void* h, double* outBuf, int len)
{
    auto* c = C(h); if (!c || !outBuf || len<=0) return 0;
    int n = (c->m->nu < len) ? (int)c->m->nu : len;
    std::memcpy(outBuf, c->d->actuator_force, n * sizeof(double));
    return n;
}

// geprojecteerde actuator-kracht op DOF's (nv): qfrc_actuator
MJW_API int __cdecl MJW_GetQfrcActuator(void* h, double* outBuf, int len)
{
    auto* c = C(h); if (!c || !outBuf || len<=0) return 0;
    int n = (c->m->nv < len) ? (int)c->m->nv : len;
    std::memcpy(outBuf, c->d->qfrc_actuator, n * sizeof(double));
    return n;
}

// externe koppel/force op DOF i toevoegen (qfrc_applied[i] += tau_or_force)
// (positieve waarde volgt MuJoCo DOF-oriëntatie; voor hinge = Nm, slide = N)
MJW_API void __cdecl MJW_ApplyDof(void* h, int dofIndex, double value)
{
    auto* c = C(h); if (!c) return;
    if (dofIndex < 0 || dofIndex >= (int)c->m->nv) return;
    c->d->qfrc_applied[dofIndex] += (mjtNum)value;
}

// alle extern aangebrachte krachten/koppels wissen
MJW_API void __cdecl MJW_ClearApplied(void* h)
{
    auto* c = C(h); if (!c) return;
    std::memset(c->d->qfrc_applied, 0, c->m->nv * sizeof(mjtNum));
}

// joint-naam → eerste DOF-index en #DOF’s (handig voor ball/free joints)
// return: 1 = ok, 0 = niet gevonden of ongeldig
MJW_API int __cdecl MJW_JointNameToDof(void* h, const char* jointName, int* firstDof, int* ndof)
{
    auto* c = C(h); if (!c || !jointName) return 0;

    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return 0;

    // type bepaalt aantal dofs
    int t = c->m->jnt_type[jid];
    int n = 0;
    switch (t) {
      case mjJNT_HINGE:  n = 1; break;
      case mjJNT_SLIDE:  n = 1; break;
      case mjJNT_BALL:   n = 3; break;
      case mjJNT_FREE:   n = 6; break;
      default:           n = 0; break;
    }
    if (n == 0) return 0;

    int adr = c->m->jnt_dofadr[jid];
    if (firstDof) *firstDof = adr;
    if (ndof)     *ndof     = n;
    return 1;
}

// --------

#include <cmath>   // for NAN

// angle (hinge rad / slide m); NaN if not found
MJW_API double __cdecl MJW_Joint1D_Angle(void* h, const char* jointName)
{
    auto* c = C(h); if (!c || !jointName) return NAN;
    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return NAN;

    int t = c->m->jnt_type[jid];
    if (t != mjJNT_HINGE && t != mjJNT_SLIDE) return NAN;

    int qpos_adr = c->m->jnt_qposadr[jid];   // <-- qpos index voor hoek/positie
    return (double)c->d->qpos[qpos_adr];
}

// angular/linear velocity (rad/s or m/s); NaN if not found
MJW_API double __cdecl MJW_Joint1D_Vel(void* h, const char* jointName)
{
    auto* c = C(h); if (!c || !jointName) return NAN;
    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return NAN;

    int t = c->m->jnt_type[jid];
    if (t != mjJNT_HINGE && t != mjJNT_SLIDE) return NAN;

    int dof_adr  = c->m->jnt_dofadr[jid];    // <-- qvel index voor snelheid
    return (double)c->d->qvel[dof_adr];
}

// ---- joint "proprioception" helpers ----

MJW_API int __cdecl MJW_GetJoint1D_State(void* h, const char* jointName,
                                         double* angle, double* velocity)
{
    auto* c = C(h); if (!c || !jointName) return 0;
    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return 0;

    int t = c->m->jnt_type[jid];
    if (t != mjJNT_HINGE && t != mjJNT_SLIDE) return 0;  // alleen 1-DOF types

    // LET OP: qpos-index ≠ qvel-index
    int qpos_adr = c->m->jnt_qposadr[jid];   // index voor d->qpos (hoek of translatie)
    int dof_adr  = c->m->jnt_dofadr[jid];    // index voor d->qvel (ω of v)

    if (angle)    *angle    = (double)c->d->qpos[qpos_adr];
    if (velocity) *velocity = (double)c->d->qvel[dof_adr];
    return 1;
}

// Generic: fill angle[0..ndof-1], vel[0..ndof-1], return ndof (1,3,6) or 0 on error.
// For ball: ndof=3 (angular velocity components), for free: ndof=6 (3 pos, 3 rot velocity style).
MJW_API int __cdecl MJW_GetJointState(void* h, const char* jointName,
                                      double* angleBuf, double* velBuf, int maxLen)
{
    auto* c = C(h); if (!c || !jointName || maxLen<=0) return 0;
    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return 0;

    int t   = c->m->jnt_type[jid];
    int adr = c->m->jnt_dofadr[jid];
    int nd  = 0;

    switch (t)
    {
      case mjJNT_HINGE:  // 1-DOF angle
      case mjJNT_SLIDE:  // 1-DOF translation
        nd = 1;
        break;
      case mjJNT_BALL:   // 3 rotational DOF
        nd = 3;
        break;
      case mjJNT_FREE:   // 6 DOF (3 pos + 3 rot velocity style)
        nd = 6;
        break;
      default:
        return 0;
    }

    if (maxLen < nd) return 0;

    // angles/positions
    for (int i=0; i<nd; ++i)
      if (angleBuf) angleBuf[i] = (double)c->d->qpos[adr + i];

    // velocities
    for (int i=0; i<nd; ++i)
      if (velBuf) velBuf[i] = (double)c->d->qvel[adr + i];

    return nd;
}

// -------

MJW_API int __cdecl MJW_DebugJoint1D(void* h, const char* jointName,
                                     int* jtype, int* qposAdr, int* dofAdr)
{
    auto* c=C(h); if(!c || !jointName) return 0;
    int jid = mj_name2id(c->m, mjOBJ_JOINT, jointName);
    if (jid < 0) return 0;
    int t   = c->m->jnt_type[jid];
    if (jtype)   *jtype = t;
    if (qposAdr) *qposAdr = c->m->jnt_qposadr[jid];
    if (dofAdr)  *dofAdr  = c->m->jnt_dofadr[jid];
    return (t==mjJNT_HINGE || t==mjJNT_SLIDE) ? 1 : 0;
}

// - Version -> rem. (redeclared)


// -

// - MJW_API int __cdecl MJW_Ping(int x) { return x + 123; }

#include <cstdio>

MJW_API void __cdecl MJW_DumpAllJoints(void* h)
{
    auto* c = C(h); 
    if (!c) { printf("No context\n"); return; }

    printf("---- JOINT DUMP ----\n");
    for (int j = 0; j < c->m->njnt; j++)
    {
        const char* name = mj_id2name(c->m, mjOBJ_JOINT, j);
        int type = c->m->jnt_type[j];
        int qp   = c->m->jnt_qposadr[j];
        int dv   = c->m->jnt_dofadr[j];
        double q = c->d->qpos[qp];
        double v = c->d->qvel[dv];
        printf("%-18s type=%d qp=%d dv=%d  q=%.4f  v=%.4f\n",
               name ? name : "?", type, qp, dv, q, v);
    }
    printf("--------------------\n");
}

// --------

// ===== Joint introspection & live data (Delphi interface) =====

// 1) aantal joints
MJW_API int __cdecl MJW_GetJointCount(void* h)
{
    auto* c = C(h); 
    if (!c) return 0;
    return (int)c->m->njnt;
}

// 2) naam van joint i
MJW_API const char* __cdecl MJW_GetJointName(void* h, int jindex)
{
    auto* c = C(h);
    if (!c) return nullptr;
    if (jindex < 0 || jindex >= (int)c->m->njnt) return nullptr;
    return mj_id2name(c->m, mjOBJ_JOINT, jindex);
}

// 3) basisinformatie over joint i
MJW_API int __cdecl MJW_GetJointInfo(void* h, int jindex,
                                     int* jtype, int* qposAdr, int* dofAdr, int* ndof)
{
    auto* c = C(h); 
    if (!c) return 0;
    if (jindex < 0 || jindex >= (int)c->m->njnt) return 0;

    int t   = c->m->jnt_type[jindex];
    int qp  = c->m->jnt_qposadr[jindex];
    int dv  = c->m->jnt_dofadr[jindex];
    int nd  = 0;
    switch (t)
    {
        case mjJNT_HINGE: nd = 1; break;
        case mjJNT_SLIDE: nd = 1; break;
        case mjJNT_BALL:  nd = 3; break;
        case mjJNT_FREE:  nd = 6; break;
        default:          nd = 0; break;
    }

    if (jtype)   *jtype   = t;
    if (qposAdr) *qposAdr = qp;
    if (dofAdr)  *dofAdr  = dv;
    if (ndof)    *ndof    = nd;
    return 1;
}

// 4) actuele waarden (hoek/snelheid) van joint i
MJW_API int __cdecl MJW_GetJointValuesByIndex(void* h, int jindex,
                                              double* angleBuf, double* velBuf, int maxLen)
{
    auto* c = C(h); 
    if (!c || maxLen <= 0) return 0;
    if (jindex < 0 || jindex >= (int)c->m->njnt) return 0;

    int t   = c->m->jnt_type[jindex];
    int qp  = c->m->jnt_qposadr[jindex];
    int dv  = c->m->jnt_dofadr[jindex];
    int nd  = 0;
    switch (t)
    {
        case mjJNT_HINGE: nd = 1; break;
        case mjJNT_SLIDE: nd = 1; break;
        case mjJNT_BALL:  nd = 3; break;
        case mjJNT_FREE:  nd = 6; break;
        default:          return 0;
    }
    if (maxLen < nd) return 0;

    for (int i = 0; i < nd; ++i)
    {
        if (angleBuf) angleBuf[i] = (double)c->d->qpos[qp + i];
        if (velBuf)   velBuf[i]   = (double)c->d->qvel[dv + i];
    }
    return nd;
}

// 5) simulatie-tijd
MJW_API double __cdecl MJW_GetSimTime(void* h)
{
    auto* c = C(h); 
    if (!c) return 0.0;
    return (double)c->d->time;
}

// 6) versie- en ping-functies (handig voor debug)
MJW_API const char* __cdecl MJW_Version() { return "MJW 1.4 (Joint Introspection API)"; }
MJW_API int __cdecl MJW_Ping(int x) { return x + 456; }

MJW_API int __cdecl MJW_ReadByAdr(void* h, int qposAdr, int dofAdr,
                                  double* angleOut, double* velOut)
{
    auto* c = C(h); if (!c) return 0;
    if (qposAdr < 0 || qposAdr >= (int)c->m->nq) return 0;
    if (dofAdr  < 0 || dofAdr  >= (int)c->m->nv) return 0;
    if (angleOut) *angleOut = (double)c->d->qpos[qposAdr];
    if (velOut)   *velOut   = (double)c->d->qvel[dofAdr];
    return 1;
}

// sizes (handig voor bounds)
MJW_API void __cdecl MJW_GetSizes(void* h, int* nq, int* nv)
{
    auto* c = C(h); if (!c) { if(nq)*nq=0; if(nv)*nv=0; return; }
    if (nq) *nq = (int)c->m->nq;
    if (nv) *nv = (int)c->m->nv;
}

// direct readers (return NaN als out-of-range)
#include <cmath>

MJW_API double __cdecl MJW_ReadQpos(void* h, int idx)
{
    auto* c = C(h); if (!c) return NAN;
    if (idx < 0 || idx >= (int)c->m->nq) return NAN;
    return (double)c->d->qpos[idx];
}

MJW_API double __cdecl MJW_ReadQvel(void* h, int idx)
{
    auto* c = C(h); if (!c) return NAN;
    if (idx < 0 || idx >= (int)c->m->nv) return NAN;
    return (double)c->d->qvel[idx];
}

// ------------

// ====== BEGIN: extra uitlees-API ======



// ====== EINDE: extra uitlees-API ======


// === PATCH 4: nieuwe exports voor muis-camera ===
MJW_API void __cdecl MJW_EnableMouseCam(void* handle, int enable)
{
    auto* c = static_cast<MJCtx*>(handle);
    if (!c || !c->win) return;

    c->mouseCamEnabled = (enable != 0);
    glfwSetWindowUserPointer(c->win, c); // nodig voor callbacks

    if (c->mouseCamEnabled) {
        double x, y; glfwGetCursorPos(c->win, &x, &y);
        c->lastX = x; c->lastY = y;
        glfwSetCursorPosCallback   (c->win, s_cursorPosCB);
        glfwSetMouseButtonCallback (c->win, s_mouseBtnCB);
        glfwSetScrollCallback      (c->win, s_scrollCB);
    } else {
        c->btnL = c->btnR = c->btnM = false;
        glfwSetCursorPosCallback   (c->win, nullptr);
        glfwSetMouseButtonCallback (c->win, nullptr);
        glfwSetScrollCallback      (c->win, nullptr);
    }
}

MJW_API void __cdecl MJW_SetMouseCamParams(void* handle, double rotScale, double panScale, double zoomScale)
{
    auto* c = static_cast<MJCtx*>(handle);
    if (!c) return;
    if (rotScale  > 0) c->rotScale  = rotScale;
    if (panScale  > 0) c->panScale  = panScale;
    if (zoomScale > 0) c->zoomScale = zoomScale;
}