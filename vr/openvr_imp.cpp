#include "stdafx.h"
#include "openvr_imp.h"
#include "utilities/Logs.h"
#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "application/application.h"
#include "input/drivermouseinput.h"
#include "utilities/Timer.h"

vr_openvr::vr_openvr()
{
    vr::EVRInitError vr_error;
    vr_system = vr::VR_Init(&vr_error, vr::VRApplication_Scene);
    if (vr_error != vr::VRInitError_None) {
        ErrorLog(std::string("failed to init openvr: ") + vr::VR_GetVRInitErrorAsEnglishDescription(vr_error));
        vr_system = nullptr;
        return;
    }
    WriteLog("openvr initialized");

    vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseSeated);
    vr::VRInput()->GetInputSourceHandle("/user/hand/left", &inputhandle_left);
    vr::VRInput()->GetInputSourceHandle("/user/hand/right", &inputhandle_right);

    std::string action_path = std::filesystem::current_path().string() + "/openvrconfig/openvr_actions.json";
    vr::VRInput()->SetActionManifestPath(action_path.c_str());

    vr::VRInput()->GetActionSetHandle("/actions/main", &actionset);
    vr::VRInput()->GetActionHandle("/actions/main/in/PrimaryAction", &primary_action);
    vr::VRInput()->GetActionHandle("/actions/main/in/SecondaryAction", &secondary_action);
    vr::VRInput()->GetActionHandle("/actions/main/in/ResetView", &resetview_action);

    hiddenarea_mesh[(size_t)vr_interface::eye_left] = create_hiddenarea_model(vr_interface::eye_left);
    hiddenarea_mesh[(size_t)vr_interface::eye_right] = create_hiddenarea_model(vr_interface::eye_right);
}

std::unique_ptr<TModel3d> vr_openvr::create_hiddenarea_model(vr_interface::eye_e e)
{
    vr::HiddenAreaMesh_t mesh = vr_system->GetHiddenAreaMesh((e == vr_interface::eye_left) ? vr::Eye_Left : vr::Eye_Right, vr::k_eHiddenAreaMesh_Standard);
    if (!mesh.unTriangleCount)
        return nullptr;

    gfx::vertex_array vertices;
    for (size_t v = 0; v < mesh.unTriangleCount * 3; v++) {
        const vr::HmdVector2_t vertex = mesh.pVertexData[v];
        vertices.push_back(gfx::basic_vertex(
                               glm::vec3(vertex.v[0], vertex.v[1], 0.0f),
                               glm::vec3(0.0f),
                               glm::vec2(0.0f)));
    }


    std::unique_ptr<TModel3d> model = std::make_unique<TModel3d>();
    model->AppendChildFromGeometry("__root", "none", vertices, gfx::index_array());
    model->Init();

    return model;
}

TModel3d* vr_openvr::get_hiddenarea_mesh(eye_e eye)
{
    return hiddenarea_mesh[(size_t)eye].get();
};

glm::ivec2 vr_openvr::get_target_size()
{
    uint32_t vr_w, vr_h;
    vr_system->GetRecommendedRenderTargetSize(&vr_w, &vr_h);
    return glm::ivec2(vr_w, vr_h);
}

viewport_proj_config vr_openvr::get_proj_config(eye_e e)
{
    vr::EVREye eye = (e == vr_interface::eye_left) ? vr::Eye_Left : vr::Eye_Right;

    float left, right, top, bottom; // tangents of half-angles from center view axis
    vr_system->GetProjectionRaw(eye, &left, &right, &top, &bottom);

    glm::mat4 eye_mat = get_matrix(vr_system->GetEyeToHeadTransform(eye));

    viewport_proj_config proj;

    proj.pe = eye_mat * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    proj.pa = eye_mat * glm::vec4(left, top, -1.0f, 1.0f);
    proj.pc = eye_mat * glm::vec4(left, bottom, -1.0f, 1.0f);
    proj.pb = eye_mat * glm::vec4(right, top, -1.0f, 1.0f);

    return proj;
}

glm::mat4 vr_openvr::get_matrix(const vr::HmdMatrix34_t &src)
{
    return glm::mat4(glm::transpose(glm::make_mat3x4((float*)(src.m))));
}

void vr_openvr::begin_frame()
{
    vr::TrackedDevicePose_t devicePose[MAX_DEVICES];
    vr::VRCompositor()->WaitGetPoses(devicePose, MAX_DEVICES, nullptr, 0);

    for (size_t i = 0; i < MAX_DEVICES; i++) {
        if (!vr_system->IsTrackedDeviceConnected(i)) {
            if (controllers[i])
                controllers[i].reset();

            continue;
        }

        glm::mat4 device_pose = get_matrix(devicePose[i].mDeviceToAbsoluteTracking);

        vr::ETrackedDeviceClass device_class = vr_system->GetTrackedDeviceClass(i);

        if (device_class == vr::TrackedDeviceClass_HMD) {
            Global.viewport_move = glm::vec3(device_pose[3]);
            Global.viewport_rotate = glm::mat3(device_pose);
        }

        if (device_class == vr::TrackedDeviceClass_Controller) {
            if (!controllers[i]) {
                controllers[i] = std::make_shared<controller_info>();

                vr::ETrackedControllerRole role = (vr::ETrackedControllerRole)vr_system->GetInt32TrackedDeviceProperty(i, vr::Prop_ControllerRoleHint_Int32);
                if (role == vr::TrackedControllerRole_LeftHand)
                    controllers[i]->inputhandle = inputhandle_left;
                else if (role == vr::TrackedControllerRole_RightHand)
                    controllers[i]->inputhandle = inputhandle_right;

                char model_name[128];
                size_t ret = vr_system->GetStringTrackedDeviceProperty(i, vr::Prop_RenderModelName_String, model_name, sizeof(model_name));

                if (ret != 0) {
                    controllers[i]->rendermodel = std::string(model_name);

                    uint32_t component_count = vr::VRRenderModels()->GetComponentCount(model_name);
                    if (component_count == 0 || !controllers[i]->inputhandle) {
                        controllers[i]->pending_components.push_back(-1);
                    } else {
                        for (size_t c = 0; c < component_count; c++)
                            controllers[i]->pending_components.push_back(c);
                    }

                    controllers[i]->model = std::make_unique<TModel3d>();
                    gfx::vertex_array vertices {
                        gfx::basic_vertex{ glm::vec3(0.005f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) },
                        gfx::basic_vertex{ glm::vec3(-0.005f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) },
                        gfx::basic_vertex{ glm::vec3(0.0f, 0.0f, -2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) },

                        gfx::basic_vertex{ glm::vec3(0.0f, 0.0f, -2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) },
                        gfx::basic_vertex{ glm::vec3(-0.005f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) },
                        gfx::basic_vertex{ glm::vec3(0.005f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f) } };
                    controllers[i]->model->AppendChildFromGeometry("__root", "none", gfx::vertex_array(), gfx::index_array());

                    controllers[i]->model->AppendChildFromGeometry("__pointer", "__root", vertices, gfx::index_array());
                }
            }

            for (size_t c = controllers[i]->pending_components.size(); c-- != 0;) {
                int32_t component = controllers[i]->pending_components[c];
                const char *rendermodel = controllers[i]->rendermodel.c_str();

                char rendermodel_name[256];
                char component_name[128];

                gfx::vertex_array vertices;
                gfx::index_array indices;

                std::string submodel_name;
                vr::RenderModel_t *model;
                vr::EVRRenderModelError ret;
                TSubModel *sm;

                if (component == -1) {
                    const char *name = controllers[i]->rendermodel.c_str();
                    strncpy(rendermodel_name, name, sizeof(rendermodel_name) - 1);
                    rendermodel_name[255] = 0;
                }
                else {
                    vr::VRRenderModels()->GetComponentName(rendermodel, component, component_name, sizeof(component_name));
                    if (!vr::VRRenderModels()->GetComponentRenderModelName(rendermodel, component_name, rendermodel_name, sizeof(rendermodel_name)))
                        goto component_done;
                }

                ret = vr::VRRenderModels()->LoadRenderModel_Async(rendermodel_name, &model);

                if (ret == vr::VRRenderModelError_Loading)
                    continue;
                else if (ret != vr::VRRenderModelError_None)
                    goto component_done;

                indices.reserve(model->unTriangleCount * 3);
                for (size_t v = 0; v < model->unTriangleCount * 3; v++)
                    indices.push_back(model->rIndexData[v]);

                vertices.reserve(model->unVertexCount);
                for (size_t v = 0; v < model->unVertexCount; v++) {
                    const vr::RenderModel_Vertex_t *vertex = &model->rVertexData[v];
                    vertices.push_back(gfx::basic_vertex(
                                           glm::vec3(vertex->vPosition.v[0], vertex->vPosition.v[1], vertex->vPosition.v[2]),
                                           glm::vec3(vertex->vNormal.v[0], vertex->vNormal.v[1], vertex->vNormal.v[2]),
                                           glm::vec2(vertex->rfTextureCoord[0], vertex->rfTextureCoord[1])));
                }

                gfx::calculate_tangents(vertices, indices, GL_TRIANGLES);

                submodel_name = std::string(component == -1 ? rendermodel_name : component_name);
                sm = controllers[i]->model->AppendChildFromGeometry(submodel_name, "__root", vertices, indices);

                if (model->diffuseTextureId >= 0)
                    controllers[i]->pending_textures.push_back(std::make_pair(sm, model->diffuseTextureId));

                vr::VRRenderModels()->FreeRenderModel(model);

                if (component != -1 && update_component(controllers[i]->rendermodel, controllers[i]->inputhandle, sm))
                    controllers[i]->active_components.push_back(sm);

component_done:
                controllers[i]->pending_components.erase(controllers[i]->pending_components.begin() + c);

                if (controllers[i]->pending_components.empty())
                    controllers[i]->model->Init();
            }

            for (size_t t = controllers[i]->pending_textures.size(); t-- != 0;) {
                TSubModel *sm = controllers[i]->pending_textures[t].first;
                const vr::TextureID_t texture_id = controllers[i]->pending_textures[t].second;

                if (GfxRenderer->Material(sm->GetMaterial())->GetTexture(0) == null_handle)
                    sm->ReplaceMaterial("internal_src:OPENVR" + std::to_string(texture_id));
                auto &tex = GfxRenderer->Texture(GfxRenderer->Material(sm->GetMaterial())->GetTexture(0));

                if (tex.is_stub()) {
                    vr::RenderModel_TextureMap_t *texturemap;
                    auto ret = vr::VRRenderModels()->LoadTexture_Async(texture_id, &texturemap);

                    if (ret == vr::VRRenderModelError_Loading)
                        continue;
                    else if (ret != vr::VRRenderModelError_None)
                        goto texture_done;

                    if (texturemap->format == vr::VRRenderModelTextureFormat_RGBA8_SRGB)
                        tex.make_from_memory(texturemap->unWidth, texturemap->unHeight, texturemap->rubTextureMapData);
                    else
                        ErrorLog("openvr texture format " + std::to_string(texturemap->format) + " not supported");

                    vr::VRRenderModels()->FreeTexture(texturemap);
                }

texture_done:
                controllers[i]->pending_textures.erase(controllers[i]->pending_textures.begin() + t);
            }

            if (controllers[i]->model) {
                glm::vec3 trans = glm::vec3(device_pose[3]);
                glm::mat3 rot = glm::mat3(device_pose);
                trans -= Global.viewport_move;

                controllers[i]->model->GetSMRoot()->ReplaceMatrix(glm::translate(glm::mat4(), trans) * glm::mat4(rot));

                for (TSubModel *component : controllers[i]->active_components)
                    update_component(controllers[i]->rendermodel, controllers[i]->inputhandle, component);
            }

            if (i == primary_controller)
                primary_controller_transform = device_pose;
        }
    }

    vr::VRActiveActionSet_t active_actionset;
    active_actionset.ulActionSet = actionset;
    active_actionset.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
    active_actionset.nPriority = 0;

    vr::VRInput()->UpdateActionState(&active_actionset, sizeof(active_actionset), 1);

    vr::InputDigitalActionData_t actiondata_pri;
    vr::VRInput()->GetDigitalActionData(primary_action, &actiondata_pri, sizeof(actiondata_pri), vr::k_ulInvalidInputValueHandle);

    vr::InputDigitalActionData_t actiondata_sec;
    vr::VRInput()->GetDigitalActionData(secondary_action, &actiondata_sec, sizeof(actiondata_sec), vr::k_ulInvalidInputValueHandle);

    vr::InputDigitalActionData_t actiondata_reset;
    vr::VRInput()->GetDigitalActionData(resetview_action, &actiondata_reset, sizeof(actiondata_reset), vr::k_ulInvalidInputValueHandle);

    if (actiondata_reset.bActive && actiondata_reset.bChanged && actiondata_reset.bState)
        reset_view();

    update_controls(actiondata_pri, actiondata_sec);
/*
    bool prev_should_pause = should_pause;
    should_pause = vr_system->ShouldApplicationPause();
    if (prev_should_pause != should_pause)
        relay.post(user_command::focuspauseset, should_pause ? 1.0 : 0.0, 0.0, GLFW_PRESS, 0);*/

    draw_controllers = !vr_system->IsSteamVRDrawingControllers();

    vr::VREvent_t event;
    while (vr_system->PollNextEvent(&event, sizeof(event)))
    {
        if (event.eventType == vr::VREvent_Quit) {
            vr_system->AcknowledgeQuit_Exiting();
            Application.queue_quit(true);
        }
    }
}

// recenters the seated tracking space on the current headset position
void vr_openvr::reset_view()
{
    if (auto *chaperone = vr::VRChaperone()) {
        chaperone->ResetZeroPose(vr::TrackingUniverseSeated);
        WriteLog("vr: seated position reset");
    }
}

// commands which move a lever to a specified position; with the vr controller these are grabbed and dragged
static bool is_lever_command(user_command const command)
{
    switch (command) {
    case user_command::jointcontrollerset:
    case user_command::mastercontrollerset:
    case user_command::secondcontrollerset:
    case user_command::trainbrakeset:
    case user_command::independentbrakeset:
        return true;
    default:
        return false;
    }
}

void vr_openvr::update_controls(vr::InputDigitalActionData_t const &primary, vr::InputDigitalActionData_t const &secondary)
{
    // react only to a button going down, so releasing one button doesn't pick a new control
    bool const primary_pressed = (primary.bChanged && primary.bState);
    bool const secondary_pressed = (secondary.bChanged && secondary.bState);
    bool const any_active = (primary.bState || secondary.bState);

    if (command_active == user_command::none && !pick_pending && (primary_pressed || secondary_pressed)) {
        bool const use_primary = primary_pressed;

        vr::InputOriginInfo_t origin;
        vr::VRInput()->GetOriginTrackedDeviceInfo(use_primary ? primary.activeOrigin : secondary.activeOrigin, &origin, sizeof(origin));
        primary_controller = origin.trackedDeviceIndex;

        pick_pending = true;
        GfxRenderer->Pick_Control_Callback([this, use_primary](TSubModel const *control, const glm::vec2 pos) {
            pick_pending = false;
            if (command_active != user_command::none || control == nullptr)
                return;

            if (use_primary && control->screen_touch_list)
                control->screen_touch_list->emplace_back(pos);

            auto const &bindings = drivermouse_input::control_bindings();
            auto const lookup = bindings.find(simulation::Train->GetLabel(control));
            if (lookup == bindings.end())
                return;

            user_command const command = (use_primary ? lookup->second.left : lookup->second.right);
            if (command == user_command::none)
                return;

            if (is_lever_command(command)) {
                // grab the lever; it then follows the vertical movement of the hand
                if (!lever.setup(command))
                    return;
                lever_start_position = lever.position();
                lever_start_height = primary_controller_transform[3].y;
                command_active = command;
                relay.post(command_active, lever.value(), 0, GLFW_PRESS, 0);
                return;
            }

            command_active = command;
            relay.post(command_active, 0, 0, GLFW_PRESS, 0);
            // same timing as with the mouse: a delay before the command starts repeating
            repeat_accumulator = -REPEAT_DELAY;
        });
        return;
    }

    if (command_active == user_command::none)
        return;

    if (!any_active) {
        // button released
        if (is_lever_command(command_active)) {
            relay.post(command_active, lever.value(), 0, GLFW_RELEASE, 0);
            lever.clear();
        }
        else {
            relay.post(command_active, 0, 0, GLFW_RELEASE, 0);
        }
        command_active = user_command::none;
        return;
    }

    if (is_lever_command(command_active)) {
        double const offset = (primary_controller_transform[3].y - lever_start_height) / LEVER_TRAVEL;
        double const previous = lever.value();
        lever.set_position(lever_start_position + offset);
        if (lever.value() != previous)
            relay.post(command_active, lever.value(), 0, GLFW_PRESS, 0);
        return;
    }

    // holding the button repeats the command, like holding the mouse button
    repeat_accumulator += Timer::GetDeltaRenderTime();
    while (repeat_accumulator > REPEAT_RATE) {
        relay.post(command_active, 0, 0, GLFW_REPEAT, 0);
        repeat_accumulator -= REPEAT_RATE;
    }
}

glm::mat4 vr_openvr::get_pick_transform()
{
    return primary_controller_transform;
}

bool vr_openvr::update_component(const std::string &rendermodel, vr::VRInputValueHandle_t handle, TSubModel *component)
{
    vr::RenderModel_ControllerMode_State_t state;
    state.bScrollWheelVisible = false;

    vr::RenderModel_ComponentState_t component_state;

    vr::VRRenderModels()->GetComponentStateForDevicePath(rendermodel.c_str(), component->pName.c_str(), handle, &state, &component_state);

    glm::mat4 component_pose = get_matrix(component_state.mTrackingToComponentRenderModel);
    bool visible = (component_state.uProperties & vr::VRComponentProperty_IsVisible);

    component->ReplaceMatrix(component_pose);
    component->iVisible = (visible ? 1 : 0);

    return !(component_state.uProperties & vr::VRComponentProperty_IsStatic);
}

void vr_openvr::submit(vr_openvr::eye_e eye, opengl_texture* tex)
{
    vr::Texture_t hmd_tex =
        { (void*)(uint64_t)tex->id,
          vr::TextureType_OpenGL,
          vr::ColorSpace_Gamma };

    vr::VRCompositor()->Submit(eye == vr_interface::eye_left ? vr::Eye_Left : vr::Eye_Right, &hmd_tex);
}

std::vector<TModel3d*> vr_openvr::get_render_models()
{
    std::vector<TModel3d*> list;

    if (!draw_controllers)
        return list;

    for (auto entry : controllers)
    {
        if (entry && entry->model && entry->pending_components.empty())
            list.push_back(entry->model.get());
    }

    return list;
}

void vr_openvr::finish_frame()
{
    vr::VRCompositor()->PostPresentHandoff();
}

vr_openvr::~vr_openvr()
{
    if (vr_system)
        vr::VR_Shutdown();
}

std::unique_ptr<vr_interface> vr_openvr::create_func()
{
    auto backend = std::make_unique<vr_openvr>();
    if (!backend->vr_system) {
        // SteamVR not running or no headset; fall back to regular desktop rendering
        ErrorLog("openvr unavailable, continuing without vr");
        return nullptr;
    }
    return backend;
}

bool vr_openvr::backend_register = vr_interface_factory::get_instance()->register_backend("openvr", vr_openvr::create_func);
