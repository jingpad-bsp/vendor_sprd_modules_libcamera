/* Copyright (c) 2012-2013, The Linux Foundataion. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef __SPRDCAMERA3FACTORY_H__
#define __SPRDCAMERA3FACTORY_H__

#include <hardware/camera3.h>

#include "SprdCamera3HWI.h"

namespace sprdcamera {

class SprdCamera3Factory {
  public:
    SprdCamera3Factory();
    virtual ~SprdCamera3Factory();
    static int get_number_of_cameras();
    static int setTorchMode(const char *, bool);
    static int get_camera_info(int camera_id, struct camera_info *info);
    static int set_callbacks(const camera_module_callbacks_t *callbacks);
    static void get_vendor_tag_ops(vendor_tag_ops_t *ops);

  private:
    int getNumberOfCameras();
    int getCameraInfo(int camera_id, struct camera_info *info);
    int getHighResolutionSize(int camera_id, struct camera_info *info);
    int cameraDeviceOpen(int camera_id, struct hw_device_t **hw_device);
    static int camera_device_open(const struct hw_module_t *module,
                                  const char *id,
                                  struct hw_device_t **hw_device);
    static bool isSingleIdExposeOnMultiCameraMode(int cameraId);
    static int multiCameraModeIdToPhyId(int cameraId);
    camera_metadata_t *mStaticMetadata;

  public:
    static struct hw_module_methods_t mModuleMethods;

  private:
    int mNumOfCameras;
    const camera_module_callbacks_t * mCameraCallbacks;
    Mutex mLock;
};

}; /*namespace sprdcamera*/

extern camera_module_t HAL_MODULE_INFO_SYM;

#endif
