#include <vision/storage/homography_storage.h>

HomographyStorage::HomographyStorage(GlobalStorage &global_storage_): image_rect_(global_storage_.image_rect_), homography_frame_(global_storage_.homography_frame_)
{
}

