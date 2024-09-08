#include "Application.h"

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <stdio.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Geometry>

#include <glad/glad.h>          // Initialize with gladLoadGL()

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

#include <igl/axis_angle_to_quat.h>
//#include <igl/png/writePNG.h>
#include <igl/stb/write_image.h>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


Application::Application()
{
    char* a[] = { "Dear ImGui GLFW+OpenGL3 example" } ;
    Application(1, a );
}


Application::Application(int argc, char* argv[])
{
    Eigen::initParallel();

    //camera rotation pan
    numFrames = 0;
	m_currentAngle = 0.0f;
	m_enableCameraRotatePan = false;
	m_enableCameraRotatePanFix = false;
	m_rotateAxis = Eigen::Vector3f(0, 1, 0);
	m_rotatePanFixAngle = 0.0f;
	m_rotatePanSpeed = 2.0f;//degree
	m_rotateTrackBallRadius = 0.2f;
	m_useOrthogonalDepressionRotationAxis = true;
	m_rotatePanDepressionAngle = 30.0f;

    m_saveGif = false;
    GIFDelay = 10; //10ms
}

Application::~Application()
{

    this->shutGL();

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    glfwDestroyWindow(m_window);
    glfwTerminate();
}
void Application::launch(int argc, char* argv[])
{
    //initialization
    init(argc, argv);

    ImGuiIO& io = ImGui::GetIO();


    // https://developer.nvidia.com/gpugems/gpugems3/part-iv-image-effects/chapter-24-importance-being-linear
    //Monitor thinks gpu output is in SRGB space, applys gamma, to correct it to linear
    // we output linear but are not converting to SRGB without some code
    // there are two options, gamma correction in the fragment shaders or GL_FRAMEBUFFER_SRGB
    // enabling GL_FRAMEBUFFER_SRGB should automatically convert it to srgb when writing in the fragment shader
    // https://learnopengl.com/Advanced-Lighting/Gamma-Correction
    // but we also need to make sure our textures are in the correct space see the weebsite above or glTexImage2D
    // https://stackoverflow.com/questions/61014348/convert-srgb-texture-to-linear-in-opengl
    if (enableSRGBConversion)
    {
        assert(glGetError() == GL_NO_ERROR);
        glEnable(GL_FRAMEBUFFER_SRGB);
        assert(glGetError() == GL_NO_ERROR);
        //std::cout << " checking target format " << std::endl;
        //GLint pi;
        //glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_BACK,GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING, &pi);
        //assert(glGetError() == GL_NO_ERROR);
        //std::cout << " found format " << pi << " while GL_LINEAR = " << GL_LINEAR << " , GL_SRGB = " << GL_SRGB << std::endl;
        // it seems it always reports linear here... https://stackoverflow.com/questions/25842211/opengl-srgb-framebuffer-oddity
    }
    // Main loop
    while (!glfwWindowShouldClose(m_window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        //ImGui
        //ImGui_ImplWin32_EnableDpiAwareness();
        ImGui_ImplOpenGL3_NewFrame();// Start the Dear ImGui frame
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuiSetting();
        //ImGui Rendering
        ImGui::Render();

        //run simulation
        runSimulation();

        //meshes and shodow
        int display_w, display_h;
        glfwGetFramebufferSize(m_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        if (enableSRGBConversion) glEnable(GL_FRAMEBUFFER_SRGB);
        //enable blending
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_CULL_FACE);

        if (m_enableShadowMap)
        {
            m_shadowMapCamera.updateMatrices();
            //start rendering into shadow map
            m_renderingShadowMap = true;
            m_shadowMapShadingSettings = m_shadingSettings;
            m_shadowMapShadingSettings.cameraPosition = m_shadowMapCamera.computeCenter();
            m_shadowMapShadingSettings.viewProjection = m_shadowMapCamera.getProjectionMatrix() * m_shadowMapCamera.getViewMatrix();
            m_meshDrawer.setShadingSettings(m_shadowMapShadingSettings);
            m_primitiveDrawer.setShadingSettings(m_shadowMapShadingSettings);

            m_shadowMap.bindForRendering();
            glViewport(0, 0, m_shadowMap.getWidth(), m_shadowMap.getHeight());
            this->renderShadowMap();
            m_shadowMap.unbindForRendering();
            m_renderingShadowMap = false;
        }
        else
        {
            m_shadowMap.clearDepth();
        }

        //start rendering main scene
        glViewport(0, 0, display_w, display_h);
        glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        //dpi to scale for the mouse operation
        float xscale, yscale;
        glfwGetWindowContentScale(m_window, &xscale, &yscale);
        m_camera.setViewport(display_w/xscale, display_h/yscale);
        m_camera.updateMatrices();

        //fixing the camera
        // SCA paper
        //m_camera.setTrackballCenter(Eigen::Vector3f(-0.237921, -0.374116, 0.28313));
        //m_camera.setAngles(Eigen::Quaternionf(0.831315, 0.155107, - 0.523541, - 0.103739));//w,x,y,z
        //m_camera.setTrackballRadius(1.51572);//1.51572  1.3

        //m_camera.setTrackballCenter(Eigen::Vector3f(0,0,0));
        //m_camera.setAngles(Eigen::Quaternionf(0.707314, -0.706887, 0.00320365, -0.00284485));//front
        //m_camera.setAngles(Eigen::Quaternionf(0.496485, -0.495405, - 0.507321, - 0.500702));//side
        //m_camera.setTrackballRadius(0.15);

        //compare macro vs. native
        //m_camera.setTrackballCenter(Eigen::Vector3f(0,0,0));
        //m_camera.setAngles(Eigen::Quaternionf(0.672234, -0.230673, - 0.219271, -0.668439 ));
        //m_camera.setTrackballRadius(0.16);
        if (m_enableCameraRotatePanFix || m_enableCameraRotatePan)
        {
            if (m_enableCameraRotatePan)
            {
                m_currentAngle += m_rotatePanSpeed / 180.0 * M_PI;

                numFrames++;
                std::cout << "Current angle " << int(m_currentAngle / M_PI * 180.0) << " with num frames " << numFrames << std::endl;
            }
            else
                m_currentAngle = m_rotatePanFixAngle / 180.0 * M_PI;

            //if (numFrames == 181)
            if (numFrames == 361)
            {
                exit(0);
            }

            //rotate 1: this is the rotation without depression angle
            //          rotate current angle with the axis
            Eigen::AngleAxisf rotateRoundOrigin(m_currentAngle, m_rotateAxis);

            //rotate 2: if we want a certain depression angle after the above rotation, we have a second rotation
            //          rotate m_rotatePanDepressionAngle with the orthogonal axis or directly the camera axis
            Eigen::Vector3f cameraPos = m_rotateAxis.unitOrthogonal();// (0.0, 0, 0.2);
            cameraPos = rotateRoundOrigin * cameraPos;
            Eigen::Vector3f axis2 = m_useOrthogonalDepressionRotationAxis ? cameraPos.cross(m_rotateAxis) : cameraPos;
            axis2.stableNormalize();
            Eigen::AngleAxisf constantRotateAbove(m_rotatePanDepressionAngle / 180.0 * M_PI, axis2);


            Eigen::Quaternionf q(constantRotateAbove * rotateRoundOrigin);

            SimOpt::Camera* camera = getCamera();
            camera->setTrackballCenter(Eigen::Vector3f(0, 0, 0));
            Eigen::Quaternionf q_inv = q.inverse();//since it has an inverse in the camera center computation, we need to inverse it first
            camera->setAngles(q_inv);
            camera->setTrackballRadius(m_rotateTrackBallRadius);

        }


        m_shadingSettings.cameraPosition = m_camera.computeCenter();
        m_shadingSettings.viewProjection = m_camera.getProjectionMatrix() * m_camera.getViewMatrix();
        //std::cout << "camera center " << m_camera.getTrackballCenter().transpose() << std::endl;
        //std::cout << "camera angles " << m_camera.getAngles().x() << " " << m_camera.getAngles().y() << " " << m_camera.getAngles().z() << " " << m_camera.getAngles().w() << std::endl;
        //std::cout << "camera radius " << m_camera.getTrackballRadius() << std::endl;

        m_camera.getViewMatrix();
        m_meshDrawer.setShadingSettings(m_shadingSettings);
        m_primitiveDrawer.setShadingSettings(m_shadingSettings);

        if (m_renderShadowMapCamera)
        {
            double scale = 0.5;
            Eigen::Matrix4f modelMatrix = m_shadowMapCamera.getViewMatrix().inverse() * Eigen::Vector4f(scale, scale, scale, 1.0).asDiagonal();
            this->setupShader(m_program, modelMatrix);
            drawObjMesh(*m_cameraMesh, m_program);
        }
        //this->setupShader(m_program, Eigen::Matrix4f::Identity());
        this->render();
        //putting it here will not draw imgui in the output
        if(m_enableCameraRotatePan)
            saveScreenRecorder();

        if (enableSRGBConversion) glDisable(GL_FRAMEBUFFER_SRGB);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
        if (io.ConfigFlags)// & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            //ImGui::UpdatePlatformWindows();
            //ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(m_window);
        
    }

    beforeExit();
}

void Application::ImGuiSetting()
{
    // Our state
    static bool show_demo_window = false;
    static bool show_viewer_window = true;

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::Checkbox("Viewer Window", &show_viewer_window);
            ImGui::Checkbox("ImGui Demo Window", &show_demo_window);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    // 3. Show another simple window.
    if (show_viewer_window)
    {
        ImGui::Begin("Viewer Window", &show_viewer_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
        ImGui::Checkbox("ImGui Demo Window", &show_demo_window);
        ImGui::ColorEdit3("clear color", (float*)&m_clearColor[0]);
        ImGui::ColorEdit3("ambient light", (float*)&m_shadingSettings.ambientLight[0]);
        ImGui::ColorEdit3("directional light", (float*)&m_shadingSettings.directionalLight[0]);
        ImGui::SliderFloat3("light direction", (float*)&m_shadingSettings.directionalLightDirection[0], -1.0f, 1.0f);
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        if (ImGui::Button("Reload Shader Cache"))
        {
            m_shaderCache->reload();
        }

        ImGui::Checkbox("Enable Shadow Map rendering", &m_enableShadowMap);
        ImGui::Checkbox("Render shadowMapCamera", &m_renderShadowMapCamera);
        m_shadowMapCameraDistanceDummy = m_shadowMapCamera.getTrackballRadius();
        if (ImGui::SliderFloat("Shadow Map Camera Distance", &m_shadowMapCameraDistanceDummy, 0.0, 1e4, "%.5f"))
        {
            m_shadowMapCamera.setTrackballRadius(m_shadowMapCameraDistanceDummy);
        }
        //ImGui::Checkbox("Render shadow map frustum", &m_renderShadowMapFrustum);
        ImGui::SliderFloat("Shadow bias", &m_shadowBias, 0.0f, 1.0f, "%.5f");
        ImGui::SliderFloat("Shadow sampling scale", &m_shadowSamplingScale, 0.0f, 1.0f, "%.5f");

        {//camera
            static bool cameraOrthogoGraphic = false;
            bool temp_cameraOrthogoGraphic = cameraOrthogoGraphic;
            ImGui::Checkbox("Camera orthogoGraphic", &cameraOrthogoGraphic);
            if(temp_cameraOrthogoGraphic!= cameraOrthogoGraphic)
                m_camera.setOrthographic(cameraOrthogoGraphic);

            ImGui::Checkbox("Camera rotate pan fix", &m_enableCameraRotatePanFix);
            ImGui::SliderFloat("Camera rotate pan fix angle", &m_rotatePanFixAngle, -180.0, 180.0, "%.3f");
            bool old_enableCameraRotationPan = m_enableCameraRotatePan;
            ImGui::Checkbox("Camera rotate pan", &m_enableCameraRotatePan);
            if(m_enableCameraRotatePan != old_enableCameraRotationPan)
                numFrames = 0;
            ImGui::InputFloat3("Camera rotate pan axis", (float*)&(m_rotateAxis[0]));
            ImGui::SliderFloat("Camera rotate pan speed(degree)", &m_rotatePanSpeed, 0.0, 5.0, "%.3f");
            ImGui::SliderFloat("Camera rotate pan radius", &m_rotateTrackBallRadius, 0.0, 15.0, "%.3f");
            ImGui::Checkbox("Use orthogonal axis for depression", &m_useOrthogonalDepressionRotationAxis);
            ImGui::SliderFloat("Camera rotate pan depression angle", &m_rotatePanDepressionAngle, -180, 180.0, "%.3f");

            /*static float x_angle = 0.0, y_angle = 0.0, z_angle = 0.0;
            ImGui::SliderAngle("x rotation ", &x_angle);
            ImGui::SliderAngle("y rotation ", &y_angle);
            ImGui::SliderAngle("z rotation ", &z_angle);

            static float axis_x[3] = { 1.0,0.0,0.0 }, axis_y[3] = { 0.0,1.0,0.0 }, axis_z[3] = { 0.0,0.0,1.0 };
            float qrot_x[4], qrot_y[4], qrot_z[4];
            igl::axis_angle_to_quat<float>(axis_x, x_angle, qrot_x);
            igl::axis_angle_to_quat<float>(axis_y, y_angle, qrot_y);
            igl::axis_angle_to_quat<float>(axis_z, z_angle, qrot_z);
            
            Eigen::Quaternion<float> quatx = { qrot_x[0],qrot_x[1],qrot_x[2],qrot_x[3] };
            Eigen::Quaternion<float> quaty = { qrot_y[0],qrot_y[1],qrot_y[2],qrot_y[3] };
            Eigen::Quaternion<float> quatz = { qrot_z[0],qrot_z[1],qrot_z[2],qrot_z[3] };

            Eigen::Quaternion<float> qua = quatx* quaty* quatz;
            m_camera.setAngles(qua);
            m_camera.setTrackballCenter(Eigen::Vector3f(0.0, 0.0, 0.0));*/
        }
        //ImGui::Image((void*)(intptr_t)m_shadowMap.getShadowTexture(), ImVec2(1280, 720), ImVec2(0, 0), ImVec2(1, 1), ImVec4(0.1f, 0.1f, 0.1f, 0.1f));
        if (ImGui::Button("Close Me"))
            show_viewer_window = false;

        ImGui::End();
    }

}

void Application::initGL()
{

    m_meshDrawer.setShaderFolder(SIM_OPT_SHADER_FOLDER);
    m_meshDrawer.setShaderCache(m_shaderCache);
    m_meshDrawer.initGL();


    m_primitiveDrawer.setShaderFolder(SIM_OPT_SHADER_FOLDER);
    m_primitiveDrawer.setShaderCache(m_shaderCache);
    m_primitiveDrawer.initGL();
}
void Application::shutGL()
{
    m_primitiveDrawer.shutGL();
    m_meshDrawer.shutGL();
}
void Application::setupShader(ProgramGL& program, const Eigen::Matrix4f& model)
{
    if (m_renderingShadowMap) ::setupShader(m_shadowMapShadingSettings, model, program);
    else
    {
        if (program.hasUniform("shadowMap"))
        {
            this->setupShaderWithShadowMap(program, model);
        }
        else
        {
            ::setupShader(m_shadingSettings, model, program);
        }
    }
}
void Application::setupShaderWithShadowMap(ProgramGL& program, const Eigen::Matrix4f& modelMatrix)
{
    ::setupShader(m_shadingSettings, modelMatrix, program);
    program.setUniform("lightViewProjection", m_shadowMapShadingSettings.viewProjection);
    program.setUniform1f("bias", m_shadowBias);
    program.setUniform1f("shadowSamplingScale", m_shadowSamplingScale);
    int unitIdx = 31;
    glActiveTexture(GL_TEXTURE0 + unitIdx);
    glBindTexture(GL_TEXTURE_2D, m_shadowMap.getShadowTexture());
    program.setUniform1i("shadowMap", unitIdx);
}


void Application::draw_buffer_RGBA(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R, Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G, Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B, Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A)
{
    
    int width, height;
    glfwGetFramebufferSize(m_window, &width, &height);
    
    R = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>(width, height);
    G = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>(width, height);
    B = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>(width, height);
    A = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>(width, height);

    GLubyte* pixels = (GLubyte*)calloc(width * height * 4, sizeof(GLubyte));
    //glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    int count = 0;
    for (unsigned j = 0; j < height; ++j)
    {
        for (unsigned i = 0; i < width; ++i)
        {
            R(i, j) = pixels[count * 4 + 0];
            G(i, j) = pixels[count * 4 + 1];
            B(i, j) = pixels[count * 4 + 2];
            A(i, j) = pixels[count * 4 + 3];
            ++count;
        }
    }
    // Clean up
    free(pixels);

    /*int width, height;
    glfwGetWindowSize(m_window, &width, &height);
    R = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> (width, height);
    G = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> (width, height);
    B = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> (width, height);
    A = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> (width, height);

    draw_buffer(R,G,B,A);*/
}

void Application::keyPressed(int key, int mod)
{
    switch (key)
    {
    case 'r':
    case 'R':
        m_camera.reset();
        std::cout << "camera reset" << std::endl;
        break;
    default:
        break;
    }

}
void Application::keyReleased(int key, int mod)
{

}
void Application::mouseMove(double xPos, double yPos)
{
    m_camera.mouseMove(xPos, yPos);
}
void Application::mouseButtonPressed(int button, int mods, double xPos, double yPos)
{
    //if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        bool translation = mods & GLFW_MOD_SHIFT;
        m_camera.mouseDown(xPos, yPos, translation);
    }
}
void Application::mouseButtonReleased(int button, int mods, double xPos, double yPos)
{
    //if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    if(button == GLFW_MOUSE_BUTTON_LEFT)
    {
        m_camera.mouseRelease();
    }
}
void Application::scrollWheel(double yOffset)
{
    double multiplier = std::pow(0.5, yOffset * 0.3);
    m_camera.zoom(multiplier);
}

bool Application::init(int argc, char* argv[])
{
    
    m_camera.setDNear(1e-2); //1e-2
    m_camera.setDFar(1e3); //1e1
    m_camera.setOrthographic(false);
    m_camera.setRotationType(SimOpt::Camera::ROTATION_TYPE_TRACKBALL);
    m_camera.setTrackballCenter(Eigen::Vector3f::Zero());
    m_camera.setTrackballRadius(1.0);

    m_shaderCache.reset(new ShaderCache);
    m_shaderCache->addIncludeFile("lighting.glsl", SIM_OPT_SHADER_FOLDER"lighting.glsl");
    m_shaderCache->addIncludeFile("lighting_shadow.glsl", SIM_OPT_SHADER_FOLDER"lighting_shadow.glsl");

    // Setup Opengl, we do it here so in the sub class we can already rely on opengl being initialized for easy use
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return false;

    // Decide GL+GLSL versions
    // GL 4.3 + GLSL 410
    const char* glsl_version = "#version 410";
    glfwWindowHint(GLFW_SAMPLES, 8);//multisample antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#ifdef __APPLE__
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    /*glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif*/

    if (enableSRGBConversion)
    {
        glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
    }

    std::string windowNameArgv;
    for (int i = 0; i < argc; ++i) {
        std::string str(argv[i]);
        windowNameArgv += " ";
        windowNameArgv += str;
    }
    
    // Create window with graphics context
     //m_window = glfwCreateWindow(2560+1080, 1440+720, argv, NULL, NULL);//1280, 720
    //m_window = glfwCreateWindow(2560, 1440, windowNameArgv.c_str(), nullptr, nullptr);
    m_window = glfwCreateWindow(1920, 1080, windowNameArgv.c_str(), nullptr, nullptr);
    //glfwSetWindowSize(m_window, 1080, 760);
    if (m_window == NULL)
        return false;
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1); // Enable vsync

    bool err = gladLoadGL() == 0;

    if (err)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return false;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    //io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    //io.ConfigViewportsNoAutoMerge = true;
    //io.ConfigViewportsNoTaskBarIcon = true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags)// & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
    float scale = 1.0;
    io.FontGlobalScale = scale;
    style.ScaleAllSizes(scale);

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    this->initGL();

    //glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height) {
    //    auto app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    //    app->resizeWindow(width, height);
    //    });

    glfwSetWindowUserPointer(m_window, this);
    glfwSetKeyCallback(m_window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        if (ImGui::GetIO().WantCaptureKeyboard || ImGui::GetIO().WantTextInput)
        {
            ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
            return;
        }

        if (action == GLFW_PRESS)
            app->keyPressed(key, mods);

        if (action == GLFW_RELEASE)
            app->keyReleased(key, mods);

        });

    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* window, int button, int action, int mods) {
        Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        double xPos, yPos;
        glfwGetCursorPos(window, &xPos, &yPos);

        if (ImGui::GetIO().WantCaptureMouse)
        {
            ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
            return;
        }

        if (action == GLFW_PRESS)
            app->mouseButtonPressed(button, mods, xPos, yPos);

        if (action == GLFW_RELEASE)
            app->mouseButtonReleased(button, mods, xPos, yPos);
        });

    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double xpos, double ypos) {
        Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));

        if (ImGui::GetIO().WantCaptureMouse)
            return;

        app->mouseMove(xpos, ypos);
        });

    glfwSetScrollCallback(m_window, [](GLFWwindow* window, double xoffset, double yoffset) {
        if (ImGui::GetIO().WantCaptureMouse)
        {
            ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
            return;
        }

        Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
        app->scrollWheel(yoffset);
        });


    m_cameraMesh.reset(new ObjMesh(SIM_OPT_RES_FOLDER"meshes/camera/Camera.obj"));
    m_program = m_shaderCache->getProgram(SIM_OPT_SHADER_FOLDER"PrimitiveDrawer.vs", SIM_OPT_SHADER_FOLDER"PrimitiveDrawer.fs");


    m_shadowMap.init(1024, 1024);

    m_shadowMapCamera.setTrackballCenter(Eigen::Vector3f::Zero());
    m_shadowMapCamera.setTrackballRadius(5.0f);
    Eigen::Quaternionf a1 = Eigen::Quaternionf::FromTwoVectors(m_shadingSettings.directionalLightDirection, -Eigen::Vector3f::UnitZ());
    m_shadowMapCamera.setAngles(a1);
    m_shadowMapCamera.setViewport(m_shadowMap.getWidth(), m_shadowMap.getHeight());
    m_shadowMapCamera.setDNear(0.01f);
    m_shadowMapCamera.setDFar(1e2f);
    m_shadowMapCamera.setOrthographic(true);

    initScreenRecorder();
    return true;
}

void Application::saveScreenRecorder()
{
    if (m_saveGif) {
        //spdlog::info("Save frame to gif");
        // Allocate temporary buffers for image
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R;
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G;
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B;
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A;
        // Save it to a PNG
        draw_buffer_RGBA(R, G, B, A);

        //igl::png::writePNG(R, G, B, A, "./output/frame_" + std::to_string(numFrames) + ".png");
        igl::stb::write_image("./output/frame_" + std::to_string(numFrames) + ".png", R, G, B, A);
        //int width, height;
        //getWindowSize(width, height);
        //std::vector<uint8_t> img(width * height * 4);//convert data
        //for (int rowI = 0; rowI < width; rowI++) {
        //    for (int colI = 0; colI < height; colI++) {
        //        int indStart = (rowI + (height - 1 - colI) * width) * 4;
        //        img[indStart] = R(rowI, colI);
        //        img[indStart + 1] = G(rowI, colI);
        //        img[indStart + 2] = B(rowI, colI);
        //        img[indStart + 3] = A(rowI, colI);
        //    }
        //}

        //GifWriteFrame(&GIFWriter, img.data(), width, height, GIFDelay);
    }
}
void Application::initScreenRecorder()
{
    if (m_saveGif)
    {

        double delay_10ms = 10.0;
        //if(m_homogenizedMeshStructure_instance != nullptr)
        //    delay_10ms = std::min(10.0, m_homogenizedMeshStructure_instance->getTimeStep() * 100.0);

        int GIFStep = static_cast<int>(std::ceil(1.0 / delay_10ms));
        GIFDelay = static_cast<int>(delay_10ms * GIFStep); // always about 3x10ms, around 33FPS

        int width, height;
        getWindowSize(width, height);

    }
}

void Application::draw_buffer(
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A)
{
    assert(R.rows() == G.rows() && G.rows() == B.rows() && B.rows() == A.rows());
    assert(R.cols() == G.cols() && G.cols() == B.cols() && B.cols() == A.cols());

    unsigned width = R.rows();
    unsigned height = R.cols();

    // https://learnopengl.com/Advanced-OpenGL/Anti-Aliasing
    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // create a multisampled color attachment texture
    unsigned int textureColorBufferMultiSampled;
    glGenTextures(1, &textureColorBufferMultiSampled);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA, width, height, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, textureColorBufferMultiSampled, 0);
    // create a (also multisampled) renderbuffer object for depth and stencil attachments
    unsigned int rbo;
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH24_STENCIL8, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // configure second post-processing framebuffer
    unsigned int intermediateFBO;
    glGenFramebuffers(1, &intermediateFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediateFBO);
    // create a color attachment texture
    unsigned int screenTexture;
    glGenTextures(1, &screenTexture);
    glBindTexture(GL_TEXTURE_2D, screenTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screenTexture, 0);	// we only need a color buffer
    assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // Clear the buffer
    glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw
    render();

    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediateFBO);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, intermediateFBO);
    // Copy back in the given Eigen matrices
    GLubyte* pixels = (GLubyte*)calloc(width * height * 4, sizeof(GLubyte));
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    // Clean up
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteTextures(1, &screenTexture);
    glDeleteTextures(1, &textureColorBufferMultiSampled);
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteFramebuffers(1, &intermediateFBO);
    glDeleteRenderbuffers(1, &rbo);

    int count = 0;
    for (unsigned j = 0; j < height; ++j)
    {
        for (unsigned i = 0; i < width; ++i)
        {
            R(i, j) = pixels[count * 4 + 0];
            G(i, j) = pixels[count * 4 + 1];
            B(i, j) = pixels[count * 4 + 2];
            A(i, j) = pixels[count * 4 + 3];
            ++count;
        }
    }
    // Clean up
    free(pixels);
}