// drmgrab: read-only dump of the framebuffers currently scanned out by DRM CRTCs/planes.
// Output: for each fb, a 32-byte header "W H PITCH BPP\n" padded, then pitch*H bytes.
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h>
struct mode_card_res { uint64_t fb_id_ptr, crtc_id_ptr, connector_id_ptr, encoder_id_ptr; uint32_t count_fbs, count_crtcs, count_connectors, count_encoders; uint32_t min_width, max_width, min_height, max_height; };
struct mode_modeinfo { uint32_t clock; uint16_t hdisplay, hsync_start, hsync_end, htotal, hskew, vdisplay, vsync_start, vsync_end, vtotal, vscan; uint32_t vrefresh, flags, type; char name[32]; };
struct mode_crtc { uint64_t set_connectors_ptr; uint32_t count_connectors, crtc_id, fb_id, x, y, gamma_size, mode_valid; struct mode_modeinfo mode; };
struct mode_fb_cmd { uint32_t fb_id, width, height, pitch, bpp, depth, handle; };
struct mode_map_dumb { uint32_t handle, pad; uint64_t offset; };
struct get_plane_res { uint64_t plane_id_ptr; uint32_t count_planes; };
struct get_plane { uint32_t plane_id, crtc_id, fb_id, possible_crtcs, gamma_size, count_format_types; uint64_t format_type_ptr; };
struct set_client_cap { uint64_t capability, value; };
#define DRM_IOWR(nr,type) _IOWR('d', nr, type)
#define GETRES  DRM_IOWR(0xA0, struct mode_card_res)
#define GETCRTC DRM_IOWR(0xA1, struct mode_crtc)
#define GETFB   DRM_IOWR(0xAD, struct mode_fb_cmd)
#define MAPDUMB DRM_IOWR(0xB3, struct mode_map_dumb)
#define GETPLANERES DRM_IOWR(0xB5, struct get_plane_res)
#define GETPLANE DRM_IOWR(0xB6, struct get_plane)
#define SETCAP _IOW('d', 0x0d, struct set_client_cap)
static void dumpfb(int fd, uint32_t fbid) {
  struct mode_fb_cmd f = {0}; f.fb_id = fbid;
  if (ioctl(fd, GETFB, &f)) { fprintf(stderr, "GETFB %u failed\n", fbid); return; }
  fprintf(stderr, "fb %u: %ux%u pitch %u bpp %u handle %u\n", fbid, f.width, f.height, f.pitch, f.bpp, f.handle);
  if (!f.handle) return;
  struct mode_map_dumb m = {0}; m.handle = f.handle;
  if (ioctl(fd, MAPDUMB, &m)) { fprintf(stderr, "MAPDUMB failed\n"); return; }
  size_t sz = (size_t)f.pitch * f.height;
  void* p = mmap(0, sz, PROT_READ, MAP_SHARED, fd, m.offset);
  if (p == MAP_FAILED) { fprintf(stderr, "mmap failed\n"); return; }
  char hdr[32]; memset(hdr, ' ', 32); snprintf(hdr, 31, "%u %u %u %u", f.width, f.height, f.pitch, f.bpp); hdr[31] = '\n';
  fwrite(hdr, 1, 32, stdout); fwrite(p, 1, sz, stdout); munmap(p, sz);
}
int main(int argc, char** argv) {
  const char* dev = argc > 1 ? argv[1] : "/dev/dri/card0";
  int fd = open(dev, O_RDWR); if (fd < 0) { perror("open"); return 1; }
  struct set_client_cap cap = {2 /*UNIVERSAL_PLANES*/, 1}; ioctl(fd, SETCAP, &cap);
  struct get_plane_res pr = {0}; uint32_t planes[64];
  if (!ioctl(fd, GETPLANERES, &pr) && pr.count_planes) {
    if (pr.count_planes > 64) pr.count_planes = 64; pr.plane_id_ptr = (uint64_t)(uintptr_t)planes;
    if (!ioctl(fd, GETPLANERES, &pr))
      for (uint32_t i = 0; i < pr.count_planes; ++i) { struct get_plane g = {0}; g.plane_id = planes[i]; if (!ioctl(fd, GETPLANE, &g) && g.fb_id) { fprintf(stderr, "plane %u crtc %u fb %u\n", g.plane_id, g.crtc_id, g.fb_id); dumpfb(fd, g.fb_id); } }
  }
  return 0;
}
