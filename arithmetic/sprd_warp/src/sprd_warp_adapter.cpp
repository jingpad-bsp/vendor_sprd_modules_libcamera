#include <stdint.h>
#include "sprd_img_warp.h"
#include "sprd_camalg_adapter.h"
#include "sprdfdapi.h"
#include "properties.h"
#include <string.h>
#include <utils/Log.h>
#include <ui/GraphicBuffer.h>

using namespace android;

#define LOG_TAG "sprd_warp_adapter"
#define WARP_LOGE(format,...) ALOGE(format, ##__VA_ARGS__)
#define WARP_LOGD(format,...) ALOGD(format, ##__VA_ARGS__)

#if defined DEFAULT_RUNTYPE_GPU
static enum camalg_run_type g_run_type[2] = {SPRD_CAMALG_RUN_TYPE_GPU, SPRD_CAMALG_RUN_TYPE_GPU};
#elif defined DEFAULT_RUNTYPE_VDSP
static enum camalg_run_type g_run_type[2] = {SPRD_CAMALG_RUN_TYPE_VDSP, SPRD_CAMALG_RUN_TYPE_VDSP};
#else
static enum camalg_run_type g_run_type[2] = {SPRD_CAMALG_RUN_TYPE_CPU, SPRD_CAMALG_RUN_TYPE_CPU};
#endif

int sprd_warp_adapter_open(img_warp_inst_t *inst, bool *isISPZoom, void *param, INST_TAG tag)
{
    int ret = 0;

    char strRunType[256];
    if (tag == WARP_CAPTURE)
        property_get("persist.vendor.cam.warp.capture.run_type", strRunType , "");
    else if (tag == WARP_PREVIEW)
        property_get("persist.vendor.cam.warp.preview.run_type", strRunType , "");

    if (!(strcmp("gpu", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_GPU;
    } else if (!(strcmp("vdsp", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_VDSP;
    } else if (!(strcmp("cpu", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_CPU;
    }

    WARP_LOGD("current run type: %d\n", g_run_type[tag]);

    if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_GPU) {
        ret = img_warp_grid_open(inst, (img_warp_param_t *)param);
        *isISPZoom = false;
    } else if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_VDSP) {
        ret = img_warp_grid_vdsp_open(inst, (img_warp_param_t *)param, tag);
        *isISPZoom = true;
    } else {
        ret = img_warp_grid_cpu_open(inst, (img_warp_param_t *)param, tag);
        *isISPZoom = false;
    }

    return ret;
}

void *GraphicBufferHandleConvert(void *graphic_buffer_handle)
{
    GraphicBuffer *gb = (GraphicBuffer *)graphic_buffer_handle;
    return gb->getNativeBuffer();
}

void GetFaceInfo(FD_HANDLE fd, img_warp_buffer_t *input, img_warp_face_param_t *face_param)
{
    WARP_LOGD("GetFaceInfo enter!\n");
    FD_OPTION opt;
    FD_IMAGE  img;
    FD_FACEINFO fdFace;

    //WARP_LOGD("FdInitOption enter!\n");
    opt.fdEnv = FD_ENV_SW;
    FdInitOption(&opt);
    opt.workMode = FD_WORKMODE_STILL;
    int min_lenth = 0;
    if (input->width > input->height)
        min_lenth = input->height;
    else
        min_lenth = input->width;
    opt.minFaceSize = min_lenth / 12;
    FdCreateDetector(&fd, &opt);

    //WARP_LOGD("init input and  Detect Face!\n");
    img.width = input->width;
    img.height = input->height;
    img.step = input->width;
    img.data = (unsigned char *)input->addr[0];
    FdDetectFace(fd, &img);

    //WARP_LOGD("FdGetFaceCount enter!\n");
    int faceCount = FdGetFaceCount(fd);
    WARP_LOGD("face count is %d and  face info!\n", faceCount);
    face_param->face_num = faceCount;
    for (int i = 0; i < faceCount; i++) {
        //get face info
        FdGetFaceInfo(fd, i, &fdFace);
        face_param->face_info[i].fd.x = fdFace.x;
        face_param->face_info[i].fd.y = fdFace.y;
        face_param->face_info[i].fd.width = fdFace.width;
        face_param->face_info[i].fd.height = fdFace.height;
        face_param->face_info[i].fa.score = fdFace.score;
        WARP_LOGD("xy is %d%d and  width/height= %d%d!, score=%d\n", fdFace.x, fdFace.y, fdFace.width, fdFace.height, fdFace.score);
    }
    FdDeleteDetector(&fd);
    //FdClearFace(fd);
}

void sprd_warp_adapter_run(img_warp_inst_t inst, img_warp_buffer_t *input, img_warp_buffer_t *output, void *param, INST_TAG tag)
{
    FD_HANDLE fd = nullptr;
    img_warp_face_param_t face_param;

    if (tag == WARP_CAPTURE)
        GetFaceInfo(fd, input, &face_param);

    if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_GPU) {
        input->graphic_handle = GraphicBufferHandleConvert(input->graphic_handle);
        output->graphic_handle = GraphicBufferHandleConvert(output->graphic_handle);
        if (tag == WARP_CAPTURE)
            img_warp_grid_update_face_info(inst, &face_param);
        img_warp_grid_run(inst, input, output, param);
    } else if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_VDSP)
        img_warp_grid_vdsp_run(inst, input, output, param);
    else {
        if (tag == WARP_CAPTURE)
            img_warp_grid_cpu_update_face_info(inst, &face_param);
        img_warp_grid_cpu_run(inst, input, output, param);
    }
}

void sprd_warp_adapter_close(img_warp_inst_t *inst, INST_TAG tag)
{
    if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_GPU)
        img_warp_grid_close(inst);
    else if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_VDSP)
        img_warp_grid_vdsp_close(inst);
    else
        img_warp_grid_cpu_close(inst);
}

bool sprd_warp_adapter_get_isISPZoom(INST_TAG tag)
{
    char strRunType[256];
    if (tag == WARP_CAPTURE)
        property_get("persist.vendor.cam.warp.capture.run_type", strRunType , "");
    else if (tag == WARP_PREVIEW)
        property_get("persist.vendor.cam.warp.preview.run_type", strRunType , "");

    if (!(strcmp("gpu", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_GPU;
    } else if (!(strcmp("vdsp", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_VDSP;
    } else if (!(strcmp("cpu", strRunType))) {
        g_run_type[tag] = SPRD_CAMALG_RUN_TYPE_CPU;
    }

    bool isISPZoom = false;
    if (g_run_type[tag] == SPRD_CAMALG_RUN_TYPE_VDSP)
        isISPZoom = true;

    return isISPZoom;
}