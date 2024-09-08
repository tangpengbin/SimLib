#include "ShadowMap.h"

ShadowMap::ShadowMap()
{
    m_fbo = 0;
    m_texture = 0;
}

ShadowMap::~ShadowMap()
{
    if (m_fbo != 0) glDeleteFramebuffers(1, &m_fbo);
    if (m_texture != 0) glDeleteTextures(1, &m_texture);
}

bool ShadowMap::init(GLuint bufferWidth, GLuint bufferHeight)
{
    assert(glGetError() == GL_NO_ERROR);

    if (m_fbo != 0) glDeleteFramebuffers(1, &m_fbo);
    if (m_texture != 0) glDeleteTextures(1, &m_texture);

    assert(glGetError() == GL_NO_ERROR);

    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, bufferWidth, bufferHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    //float* data = new float[bufferWidth * bufferHeight];
    //for (int i = 0; i < bufferWidth * bufferHeight; i++)
    //{
    //    data[i] = rand() / 1000.f;
    //}
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, bufferWidth, bufferHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, data);
    //delete[] data;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL); // WARNING: this presents usual samplling from the texture (functions not specific to shadow mapping), i.e. imgui::Image wont work either
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GLfloat borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    assert(glGetError() == GL_NO_ERROR);

    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_texture, 0);

    // Disable writes to the color buffer
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    assert(glGetError() == GL_NO_ERROR);

    GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
        return false;
    }
    this->m_bufferHeight = bufferHeight;
    this->m_bufferWidth= bufferWidth;

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    assert(glGetError() == GL_NO_ERROR);

    return true;
}
void ShadowMap::bindForRendering()
{
    assert(glGetError() == GL_NO_ERROR);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    assert(glGetError() == GL_NO_ERROR);

}
void ShadowMap::unbindForRendering()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
void ShadowMap::clearDepth()
{
    assert(glGetError() == GL_NO_ERROR);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glClearDepth(1.0);
    glClear(GL_DEPTH_BUFFER_BIT);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    assert(glGetError() == GL_NO_ERROR);
}