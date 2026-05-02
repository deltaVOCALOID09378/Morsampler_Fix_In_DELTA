// Made And Checked By DELTA SYNTH & Gemini AI | deltaVOCALOID09378, Kanru Hua & llsm developers
// Version: 0.3.0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <ciglet/ciglet.h>
#include <libllsm/llsm.h>
#include <libpyin/pyin.h>

const char* version = "0.3.0";

// การประมาณค่าในช่วงแบบวงกลมสำหรับค่าเรเดียนสองค่า (Circular Interpolation)
static FP_TYPE linterpc(FP_TYPE a, FP_TYPE b, FP_TYPE ratio) {
  FP_TYPE ax = cos_2(a);
  FP_TYPE ay = sin_2(a);
  FP_TYPE bx = cos_2(b);
  FP_TYPE by = sin_2(b);
  FP_TYPE cx = linterp(ax, bx, ratio);
  FP_TYPE cy = linterp(ay, by, ratio);
  return atan2(cy, cx);
}

// การประมาณค่าเฟรมสเปกตรัมของเสียง (Noise/Magnitude Frame Interpolation)
static void interp_nmframe(llsm_nmframe* dst, llsm_nmframe* src,
  FP_TYPE ratio, int dst_voiced, int src_voiced) {
  for(int i = 0; i < dst -> npsd; i ++)
    dst -> psd[i] = linterp(dst -> psd[i], src -> psd[i], ratio);

  for(int b = 0; b < dst -> nchannel; b ++) {
    llsm_hmframe* srceenv = src -> eenv[b];
    llsm_hmframe* dsteenv = dst -> eenv[b];
    dst -> edc[b] = linterp(dst -> edc[b], src -> edc[b], ratio);
    int b_minnhar = min(srceenv -> nhar, dsteenv -> nhar);
    int b_maxnhar = max(srceenv -> nhar, dsteenv -> nhar);
    if(dsteenv -> nhar < b_maxnhar) {
      dsteenv -> ampl = realloc(dsteenv -> ampl, sizeof(FP_TYPE) * b_maxnhar);
      dsteenv -> phse = realloc(dsteenv -> phse, sizeof(FP_TYPE) * b_maxnhar);
    }
    for(int i = 0; i < b_minnhar; i ++) {
      dsteenv -> ampl[i] =
        linterp(dsteenv -> ampl[i], srceenv -> ampl[i], ratio);
      dsteenv -> phse[i] =
        linterpc(dsteenv -> phse[i], srceenv -> phse[i], ratio);
    }
    if(b_maxnhar == srceenv -> nhar) {
      for(int i = b_minnhar; i < b_maxnhar; i ++) {
        dsteenv -> ampl[i] = srceenv -> ampl[i];
        dsteenv -> phse[i] = srceenv -> phse[i];
      }
    }
    dsteenv -> nhar = b_maxnhar;
  }
}

// บันทึกการตั้งค่าลงไฟล์
int write_conf(FILE* f, llsm_aoptions* conf) {
    fwrite(&conf->thop, sizeof(FP_TYPE), 1, f);
    fwrite(&conf->maxnhar, sizeof(int), 1, f);
    fwrite(&conf->maxnhar_e, sizeof(int), 1, f);
    fwrite(&conf->npsd, sizeof(int), 1, f);
    fwrite(&conf->nchannel, sizeof(int), 1, f);
    int chanfreq_len = conf->nchannel;
    fwrite(&chanfreq_len, sizeof(int), 1, f);
    fwrite(conf->chanfreq, sizeof(FP_TYPE), chanfreq_len, f);
    fwrite(&conf->lip_radius, sizeof(FP_TYPE), 1, f);
    fwrite(&conf->f0_refine, sizeof(FP_TYPE), 1, f);
    fwrite(&conf->hm_method, sizeof(int), 1, f);
    fwrite(&conf->rel_winsize, sizeof(FP_TYPE), 1, f);
    return 0;
}

// อ่านการตั้งค่าจากไฟล์
int read_conf(FILE* f, llsm_aoptions* opt) {
    fread(&opt->thop, sizeof(FP_TYPE), 1, f);
    fread(&opt->maxnhar, sizeof(int), 1, f);
    fread(&opt->maxnhar_e, sizeof(int), 1, f);
    fread(&opt->npsd, sizeof(int), 1, f);
    fread(&opt->nchannel, sizeof(int), 1, f);
    int chanfreq_len;
    fread(&chanfreq_len, sizeof(int), 1, f);
    opt->chanfreq = malloc(sizeof(FP_TYPE) * chanfreq_len);
    if (!opt->chanfreq) return -1;
    fread(opt->chanfreq, sizeof(FP_TYPE), chanfreq_len, f);
    fread(&opt->lip_radius, sizeof(FP_TYPE), 1, f);
    fread(&opt->f0_refine, sizeof(FP_TYPE), 1, f);
    fread(&opt->hm_method, sizeof(int), 1, f);
    fread(&opt->rel_winsize, sizeof(FP_TYPE), 1, f);
    return 0;
}

// บันทึกข้อมูล LLSM ลงแคชไฟล์เพื่อลดเวลาการประมวลผลซ้ำ
int save_llsm(llsm_chunk* chunk, const char* filename, llsm_aoptions* conf, int* fs, int* nbit) {
  FILE* f = fopen(filename, "wb");
  if (!f) return -1;

  // ส่วนหัวไฟล์ (Header)
  fwrite("LLSM2", 1, 5, f);
  int version = 1;
  fwrite(&version, sizeof(int), 1, f);

  // จำนวนเฟรม
  int* nfrm = llsm_container_get(chunk->conf, LLSM_CONF_NFRM);
  fwrite(nfrm, sizeof(int), 1, f);
  fwrite(fs, sizeof(int), 1, f);
  fwrite(nbit, sizeof(int), 1, f);

  // บันทึกค่าการตั้งค่า
  write_conf(f, conf);

  // ข้อมูลเฟรมเสียง
  for (int i = 0; i < *nfrm; ++i) {
    llsm_container* frame = chunk->frames[i];

    // ความถี่พื้นฐาน (F0)
    FP_TYPE* f0 = llsm_container_get(frame, LLSM_FRAME_F0);
    fwrite(f0, sizeof(FP_TYPE), 1, f);

    // เฟรมฮาร์มอนิก (HM Frame)
    llsm_hmframe* hm = llsm_container_get(frame, LLSM_FRAME_HM);
    fwrite(&hm->nhar, sizeof(int), 1, f);
    fwrite(hm->ampl, sizeof(FP_TYPE), hm->nhar, f);
    fwrite(hm->phse, sizeof(FP_TYPE), hm->nhar, f);

    // เฟรมสเปกตรัม (NM Frame)
    llsm_nmframe* nm = llsm_container_get(frame, LLSM_FRAME_NM);
    fwrite(&nm->npsd, sizeof(int), 1, f);
    fwrite(nm->psd, sizeof(FP_TYPE), nm->npsd, f);

    fwrite(&nm->nchannel, sizeof(int), 1, f);
    for (int j = 0; j < nm->nchannel; ++j) {
      fwrite(&nm->edc[j], sizeof(FP_TYPE), 1, f);

      llsm_hmframe* eenv = nm->eenv[j];
      fwrite(&eenv->nhar, sizeof(int), 1, f);
      fwrite(eenv->ampl, sizeof(FP_TYPE), eenv->nhar, f);
      fwrite(eenv->phse, sizeof(FP_TYPE), eenv->nhar, f);
    }
  }

  fclose(f);
  return 0;
}

// อ่านข้อมูล LLSM จากแคชไฟล์
llsm_chunk* read_llsm(const char* filename, int* nfrm, int* fs, int* nbit) {
  FILE* f = fopen(filename, "rb");
  if (!f) return NULL;

  char header[5];
  fread(header, 1, 5, f);
  if (strncmp(header, "LLSM2", 5) != 0) { fclose(f); return NULL; }

  int version;
  fread(&version, sizeof(int), 1, f);
  if (version != 1) { fclose(f); return NULL; }
  fread(nfrm, sizeof(int), 1, f);
  fread(fs, sizeof(int), 1, f);
  fread(nbit, sizeof(int), 1, f);
  
  // อ่านการตั้งค่า
  llsm_aoptions* aopt = llsm_create_aoptions();
  int conf_r = read_conf(f, aopt);
  if (conf_r != 0) {
    llsm_delete_aoptions(aopt);
    fclose(f);
    return NULL;
  }
  llsm_container* conf = llsm_aoptions_toconf(aopt, 44100.0 / 2);
  llsm_container_attach(conf, LLSM_CONF_NFRM,
    llsm_create_int(*nfrm), llsm_delete_int, llsm_copy_int);
  llsm_chunk* chunk = llsm_create_chunk(conf, *nfrm);
  
  for (int i = 0; i < *nfrm; ++i) {
    llsm_container* frame = llsm_create_frame(0, 0, 0, 0); // โครงสร้างจำลอง

    // ความถี่พื้นฐาน (F0)
    FP_TYPE* f0 = malloc(sizeof(FP_TYPE));
    fread(f0, sizeof(FP_TYPE), 1, f);
    llsm_container_attach(frame, LLSM_FRAME_F0, f0, free, llsm_copy_fp);

    // ฮาร์มอนิก (HM)
    int nhar;
    fread(&nhar, sizeof(int), 1, f);
    llsm_hmframe* hm = llsm_create_hmframe(nhar);
    fread(hm->ampl, sizeof(FP_TYPE), nhar, f);
    fread(hm->phse, sizeof(FP_TYPE), nhar, f);
    llsm_container_attach(frame, LLSM_FRAME_HM, hm, llsm_delete_hmframe, llsm_copy_hmframe);

    // สเปกตรัม (NM)
    llsm_nmframe* nm = malloc(sizeof(llsm_nmframe));
    fread(&nm->npsd, sizeof(int), 1, f);
    nm->psd = malloc(sizeof(FP_TYPE) * nm->npsd);
    fread(nm->psd, sizeof(FP_TYPE), nm->npsd, f);

    fread(&nm->nchannel, sizeof(int), 1, f);
    nm->edc = malloc(sizeof(FP_TYPE) * nm->nchannel);
    nm->eenv = malloc(sizeof(llsm_hmframe*) * nm->nchannel);

    for (int j = 0; j < nm->nchannel; ++j) {
      fread(&nm->edc[j], sizeof(FP_TYPE), 1, f);
      int nhar_e;
      fread(&nhar_e, sizeof(int), 1, f);
      llsm_hmframe* eenv = llsm_create_hmframe(nhar_e);
      fread(eenv->ampl, sizeof(FP_TYPE), nhar_e, f);
      fread(eenv->phse, sizeof(FP_TYPE), nhar_e, f);
      nm->eenv[j] = eenv;
    }

    llsm_container_attach(frame, LLSM_FRAME_NM, nm, llsm_delete_nmframe, llsm_copy_nmframe);
    chunk->frames[i] = frame;
  }

  fclose(f);
  return chunk;
}

#define LOG2DB (20.0 / 2.3025851)
#define mag2db(x) (log_2(x) * LOG2DB)

// ผสานเฟรม LLSM (dst <- (dst &> src))
static void interp_llsm_frame(llsm_container* dst, llsm_container* src,
  FP_TYPE ratio) {
# define EPS 1e-8
  FP_TYPE dst_f0 = *((FP_TYPE*)llsm_container_get(dst, LLSM_FRAME_F0));
  FP_TYPE src_f0 = *((FP_TYPE*)llsm_container_get(src, LLSM_FRAME_F0));
  llsm_nmframe* dst_nm = llsm_container_get(dst, LLSM_FRAME_NM);
  llsm_nmframe* src_nm = llsm_container_get(src, LLSM_FRAME_NM);
  FP_TYPE* src_rd = llsm_container_get(src, LLSM_FRAME_RD);
  FP_TYPE* dst_rd = llsm_container_get(dst, LLSM_FRAME_RD);
  FP_TYPE* dst_vsphse = llsm_container_get(dst, LLSM_FRAME_VSPHSE);
  FP_TYPE* src_vsphse = llsm_container_get(src, LLSM_FRAME_VSPHSE);
  FP_TYPE* dst_vtmagn = llsm_container_get(dst, LLSM_FRAME_VTMAGN);
  FP_TYPE* src_vtmagn = llsm_container_get(src, LLSM_FRAME_VTMAGN);

  // ดึงความถี่ของเฟรมที่มีการออกเสียงเสมอ
  llsm_container* voiced = dst_f0 <= 0 && src_f0 <= 0 ? NULL :
    (src_f0 > 0 ? src : dst);
  int bothvoiced = dst_f0 > 0 && src_f0 > 0;

  int dstnhar = dst_vsphse == NULL ? 0 : llsm_fparray_length(dst_vsphse);
  int srcnhar = src_vsphse == NULL ? 0 : llsm_fparray_length(src_vsphse);
  int maxnhar = max(dstnhar, srcnhar);
  int minnhar = min(dstnhar, srcnhar);

  if(! bothvoiced && voiced == src) {
    llsm_container_attach(dst, LLSM_FRAME_F0, llsm_create_fp(src_f0),
      llsm_delete_fp, llsm_copy_fp);
    llsm_container_attach(dst, LLSM_FRAME_RD, llsm_create_fp(*src_rd),
      llsm_delete_fp, llsm_copy_fp);
  } else
  if(voiced == NULL) {
    llsm_container_attach(dst, LLSM_FRAME_F0, llsm_create_fp(0),
      llsm_delete_fp, llsm_copy_fp);
    llsm_container_attach(dst, LLSM_FRAME_RD, llsm_create_fp(1.0),
      llsm_delete_fp, llsm_copy_fp);
  }
  int nspec = dst_vtmagn != NULL ? llsm_fparray_length(dst_vtmagn) :
    (src_vtmagn != NULL ? llsm_fparray_length(src_vtmagn) : 0);

  if(bothvoiced) {
    llsm_container_attach(dst, LLSM_FRAME_F0, llsm_create_fp(
      linterp(dst_f0, src_f0, ratio)), llsm_delete_fp, llsm_copy_fp);
    llsm_container_attach(dst, LLSM_FRAME_RD, llsm_create_fp(
      linterp(*dst_rd, *src_rd, ratio)), llsm_delete_fp, llsm_copy_fp);

    FP_TYPE* vsphse = llsm_create_fparray(maxnhar);
    FP_TYPE* vtmagn = llsm_create_fparray(nspec);
    for(int i = 0; i < minnhar; i ++)
      vsphse[i] = linterpc(dst_vsphse[i], src_vsphse[i], ratio);
    for(int i = 0; i < nspec; i ++)
      vtmagn[i] = linterp(dst_vtmagn[i], src_vtmagn[i], ratio);
    if(dstnhar < srcnhar)
      for(int i = minnhar; i < maxnhar; i ++)
        vsphse[i] = src_vsphse[i];

    dst_vsphse = vsphse;
    dst_vtmagn = vtmagn;
    llsm_container_attach(dst, LLSM_FRAME_VSPHSE, dst_vsphse,
      llsm_delete_fparray, llsm_copy_fparray);
    llsm_container_attach(dst, LLSM_FRAME_VTMAGN, dst_vtmagn,
      llsm_delete_fparray, llsm_copy_fparray);
  } else if(voiced == src) {
    dst_vsphse = llsm_copy_fparray(src_vsphse);
    dst_vtmagn = llsm_copy_fparray(src_vtmagn);
    llsm_container_attach(dst, LLSM_FRAME_VSPHSE, dst_vsphse,
      llsm_delete_fparray, llsm_copy_fparray);
    llsm_container_attach(dst, LLSM_FRAME_VTMAGN, dst_vtmagn,
      llsm_delete_fparray, llsm_copy_fparray);
    FP_TYPE fade = mag2db(max(EPS, ratio));
    for(int i = 0; i < nspec; i ++) dst_vtmagn[i] += fade;
  } else {
    FP_TYPE fade = mag2db(max(EPS, 1.0 - ratio));
    for(int i = 0; i < nspec; i ++) dst_vtmagn[i] += fade;
  }
  for(int i = 0; i < nspec; i ++) dst_vtmagn[i] = max(-80, dst_vtmagn[i]);

  interp_nmframe(dst_nm, src_nm, ratio, dst_f0 > 0, src_f0 > 0);
# undef EPS
}

// การถอดรหัส Base64 สำหรับค่าระดับเสียงของ UTAU
int base64decoderForUtau(char x, char y)
{
	int ans1 = 0, ans2 = 0, ans;

	if(x=='+') ans1 = 62;
	if(x=='/') ans1 = 63;
	if(x>='0' && x <= '9') ans1 = x+4;
	if(x>='A' && x <= 'Z') ans1 = x-65;
	if(x>='a' && x <= 'z') ans1 = x-71;

	if(y=='+') ans2 = 62;
	if(y=='/') ans2 = 63;
	if(y>='0' && y <= '9') ans2 = y+4;
	if(y>='A' && y <= 'Z') ans2 = y-65;
	if(y>='a' && y <= 'z') ans2 = y-71;

	ans = (ans1<<6) | ans2;
	if(ans >= 2048) ans -= 4096;
	return ans;
}

// การอ่านเส้นกราฟระดับเสียง (Pitch Contour)
int getF0Contour(char *input, double *output)
{
	int i, j, count, length;
	i = 0;
	count = 0;
	double tmp;
    
	tmp = 0.0;
	while(input[i] != '\0')
	{
		if(input[i] == '#')
		{ 
			length = 0;
			for(j = i+1;input[j]!='#';j++)
			{
				length = length*10 + input[j]-'0';
			}
			i = j+1;
			for(j = 0;j < length;j++)
			{
				output[count++] = tmp;
			}
		}
		else
		{
			tmp = base64decoderForUtau(input[i], input[i+1]);
			output[count++] = tmp;
			i+=2;
		}
	}

	return count;
}

// ค่าเฉลี่ยความถี่ (พอร์ตโค้ดมาจาก world4utau.cpp โดย 飴屋／菖蒲氏)
double getFreqAvg(double f0[], int tLen)
{
	int i, j;
	double value = 0, r;
	double p[6], q;
	double freq_avg = 0;
	double base_value = 0;
	for (i = 0; i < tLen; i++)
	{
		value = f0[i];
		if (value < 1000.0 && value > 55.0)
		{
			r = 1.0;
			// ให้น้ำหนักกับค่าที่ใกล้เคียงกันต่อเนื่อง
			for (j = 0; j <= 5; j++)
			{
				if (i > j) {
					q = f0[i - j - 1] - value;
					p[j] = value / (value + q * q);
				} else {
					p[j] = 1/(1 + value);
				}
				r *= p[j];
			}
			freq_avg += value * r;
			base_value += r;
		}
	}
	if (base_value > 0) freq_avg /= base_value;
	return freq_avg;
}

// การแปลงชื่อตัวโน้ตเป็นค่า MIDI
static int parse_note_to_midi(const char *note_str) {
    int base_note = -1;
    switch (toupper(note_str[0])) {
        case 'C': base_note = 0; break;
        case 'D': base_note = 2; break;
        case 'E': base_note = 4; break;
        case 'F': base_note = 5; break;
        case 'G': base_note = 7; break;
        case 'A': base_note = 9; break;
        case 'B': base_note = 11; break;
        default: return -1;  // โน้ตไม่ถูกต้อง
    }

    int offset = 1;
    if (note_str[offset] == '#') {
        base_note += 1;
        offset++;
    } else if (note_str[offset] == 'b') {
        base_note -= 1;
        offset++;
    }

    int octave = atoi(note_str + offset);
    int midi_note = (octave + 1) * 12 + base_note;
    return midi_note;
}

// การแปลงชื่อตัวโน้ตเป็นค่าความถี่ (Hz)
float note_to_frequency(const char *note_str) {
    int midi = parse_note_to_midi(note_str);
    if (midi < 0) return -1.0f;  // ค่าอินพุตไม่ถูกต้อง
    return (float)(440.0 * pow(2.0, (midi - 69) / 12.0));
}

// การแปลงค่า Cents เป็น Hz Offset
void convert_cents_to_hz_offset(const double* cents, int cents_len,
                                int nfrm, int nhop, int fs, float tempo,
                                float* out_ratio_offset) {

    const float frame_duration_sec = (float)nhop / (float)fs;

    // ช่วง PIT grid จาก world4utau (วินาทีต่อส่วน PIT)
    const float pit_interval_sec = (60.0f / 96.0f) / tempo;  

    for (int i = 0; i < nfrm; ++i) {
        float time_sec = i * frame_duration_sec;

        float idx = time_sec / pit_interval_sec;

        int i0 = (int)idx;
        if (i0 < 0) i0 = 0;
        if (i0 >= cents_len) i0 = cents_len - 1;

        int i1 = i0 + 1;
        if (i1 >= cents_len) i1 = cents_len - 1;

        float frac = idx - (float)i0;

        float cents_interp = (float)(cents[i0] * (1.0f - frac) + cents[i1] * frac);

        float ratio = powf(2.0f, cents_interp / 1200.0f);
        out_ratio_offset[i] = ratio - 1.0f; // คืนค่าเป็นอัตราส่วนชดเชย (ไม่ได้เป็นหน่วย Hz โดยตรง)
    }
}

FP_TYPE* convert_env_to_vol_arr(int* p, int* v, int nfrm) {
    return NULL; // TODO: สำหรับการพัฒนาในอนาคต
}

// การปรับความเร็วของช่วงพยัญชนะต้น (Consonant Velocity)
void apply_velocity(llsm_chunk* chunk, float velocity, int* consonant_frames, int total_frames) {
    int consonant_frames_old = *consonant_frames;

    if (total_frames <= consonant_frames_old + 1) {
        printf("[ระบบ] ข้อผิดพลาดในการปรับ Velocity ไม่สามารถดำเนินการได้เนื่องจากจำนวนเฟรมน้อยเกินไป\n");
        return;
    }
        
    int consonant_frames_new =
        (int)(consonant_frames_old * velocity + 0.5f);

    // ป้องกันค่าหลุดกรอบ
    if (consonant_frames_new < 1) {
        consonant_frames_new = 1;
    }
    if (consonant_frames_new > total_frames - 1) {
      consonant_frames_new = total_frames - 1;
    }

    *consonant_frames = consonant_frames_new;

    // สร้างข้อมูลจำลองสำหรับส่วนของพยัญชนะต้นที่มีการ Resample
    llsm_chunk* tmp = llsm_create_chunk(chunk->conf, consonant_frames_new);
    
    for (int i = 0; i < consonant_frames_new; i++) {
        FP_TYPE mapped =
            (FP_TYPE)i * consonant_frames_old / consonant_frames_new;
        int base = (int)mapped;
        FP_TYPE ratio = mapped - base;

        base = min(base, consonant_frames_old - 2);
        if (base < 0) base = 0;

        tmp->frames[i] = llsm_copy_container(chunk->frames[base]);
        interp_llsm_frame(tmp->frames[i], chunk->frames[base + 1], ratio);

        FP_TYPE* resvec =
            llsm_container_get(chunk->frames[base], LLSM_FRAME_PSDRES);
        if (resvec != NULL) {
            llsm_container_attach(tmp->frames[i], LLSM_FRAME_PSDRES,
                                  llsm_copy_fparray(resvec),
                                  llsm_delete_fparray,
                                  llsm_copy_fparray);
        }
    }

    // คัดลอกข้อมูลพยัญชนะต้นที่ปรับปรุงแล้วกลับสู่โครงสร้างหลัก (Deep Copy)
    for (int i = 0; i < consonant_frames_new; i++) {
        if (chunk->frames[i])
            llsm_delete_container(chunk->frames[i]);
        chunk->frames[i] = llsm_copy_container(tmp->frames[i]);
    }

    // --- ส่วนของสระ (Vowel) ภายในตัวอย่างเสียง ---
    int vowel_frames_old = total_frames - consonant_frames_old;
    int vowel_frames_new = total_frames - consonant_frames_new;

    for (int i = 0; i < vowel_frames_new; i++) {
        int dst_idx = consonant_frames_new + i;
        int old_idx = consonant_frames_old +
                      (int)(i *
                            ((float)vowel_frames_old / vowel_frames_new));
        if (old_idx >= total_frames)
            old_idx = total_frames - 1;

        llsm_container* src       = chunk->frames[old_idx];
        llsm_container* new_frame = llsm_copy_container(src);

        if (chunk->frames[dst_idx])
            llsm_delete_container(chunk->frames[dst_idx]);

        chunk->frames[dst_idx] = new_frame;
    }

    // ล้างข้อมูลส่วนหาง (Tail) ที่เหลือทิ้ง
    for (int i = consonant_frames_new + vowel_frames_new;
         i < total_frames;
         i++) {
        if (chunk->frames[i]) {
            llsm_delete_container(chunk->frames[i]);
            chunk->frames[i] = llsm_create_frame(0, 0, 0, 0);
        }
    }

    llsm_delete_chunk(tmp);
}

// การปรับค่าความตึงเครียดของเสียง (Tension Parameter)
// ยิ่งความตึงเครียดสูง เสียงในย่านฮาร์มอนิกความถี่สูงจะถูกขยายให้ชัดเจนขึ้น
void apply_tension(llsm_chunk* chunk, FP_TYPE tension) {
    int* nfrm_p = llsm_container_get(chunk->conf, LLSM_CONF_NFRM);
    if (!nfrm_p) return;

    // ทำแผนที่ช่วงค่า [-100,100] -> [-1,1]
    const FP_TYPE t = tension / (FP_TYPE)100.0;

    // ความแรงของการเอียงสเปกตรัมโดยรวม (ในหน่วย dB)
    const FP_TYPE slope_db = (FP_TYPE)32.0 * t;

    // พารามิเตอร์กำหนดรูปทรง
    const FP_TYPE pivot = (FP_TYPE)0.25;          // จุดกึ่งกลาง
    const FP_TYPE alpha = (FP_TYPE)2.6;           // ความโค้งงอ
    const FP_TYPE eps   = (FP_TYPE)1e-12;

    for (int i = 0; i < *nfrm_p; ++i) {
        llsm_hmframe* hm = llsm_container_get(chunk->frames[i], LLSM_FRAME_HM);
        if (!hm || !hm->ampl || hm->nhar <= 0) continue;

        // ประเมินพลังงานเริ่มต้น (Pre-tilt energy) เพื่อปรับสมดุล
        FP_TYPE sum0 = 0;
        for (int j = 0; j < hm->nhar; ++j) sum0 += hm->ampl[j];

        for (int j = 0; j < hm->nhar; ++j) {
            FP_TYPE w = (hm->nhar > 1) ? (FP_TYPE)j / (FP_TYPE)(hm->nhar - 1) : 0;
            FP_TYPE w_eased = (FP_TYPE)0.5 - (FP_TYPE)0.5 * (FP_TYPE)cos(M_PI * w); // Cosine ease

            FP_TYPE h = (FP_TYPE)tanh(alpha * (w_eased - pivot));   
            FP_TYPE g_db = slope_db * h;                             
            FP_TYPE a = hm->ampl[j];
            FP_TYPE adb = (FP_TYPE)20.0 * (FP_TYPE)log10(a + eps);
            adb += g_db;
            FP_TYPE anew = (FP_TYPE)pow((FP_TYPE)10.0, adb / (FP_TYPE)20.0);

            if (anew > 1.0) anew = 1.0;
            if (anew < 0.0) anew = 0.0;
            hm->ampl[j] = anew;
        }

        // คงระดับความดังรวมเอาไว้ให้เทียบเท่ากับของเดิม (Energy Preservation)
        FP_TYPE sum1 = 0;
        for (int j = 0; j < hm->nhar; ++j) sum1 += hm->ampl[j];
        if (sum0 > 0 && sum1 > 0) {
            FP_TYPE k = sum0 / sum1;                 
            for (int j = 0; j < hm->nhar; ++j) {
                FP_TYPE v = hm->ampl[j] * k;
                hm->ampl[j] = v > 1.0 ? 1.0 : v;
            }
        }
    }
}

// ฟังก์ชันดึงค่าระดับเสียงจากพื้นที่ที่กำหนด
FP_TYPE* get_pitch_from_area(llsm_chunk* chunk, int start, int end) {
  int* nfrm_p = llsm_container_get(chunk->conf, LLSM_CONF_NFRM);
  if (!nfrm_p) return NULL;

  FP_TYPE* pitch = malloc(sizeof(FP_TYPE) * (end - start));
  if (!pitch) return NULL;

  for (int i = start; i < end; ++i) {
    FP_TYPE f0 = *((FP_TYPE*)llsm_container_get(chunk->frames[i], LLSM_FRAME_F0));
    pitch[i - start] = f0;
  }

  return pitch;
}

// การแทรกค่าระดับเสียงให้เข้ากัน (Pitch Interpolation)
FP_TYPE* interp_pitch_to_pitd(FP_TYPE* pitch, int old_nfrm, int new_nfrm) {
  FP_TYPE* new_pitch = malloc(sizeof(FP_TYPE) * new_nfrm);
  if (!new_pitch) return NULL;

  for (int i = 0; i < new_nfrm; ++i) {
    float idx = (float)i * (float)(old_nfrm - 1) / (float)(new_nfrm - 1);
    int idx0 = (int)idx;
    int idx1 = min(idx0 + 1, old_nfrm - 1);
    float frac = idx - (float)idx0;
    new_pitch[i] = pitch[idx0] * (1.0f - frac) + pitch[idx1] * frac;
  }
  
  // หาค่ามัธยฐานโดยละเว้นค่า 0
  FP_TYPE* nonzero = malloc(sizeof(FP_TYPE) * new_nfrm);
  int nz_count = 0;
  for (int i = 0; i < new_nfrm; ++i) {
    if (new_pitch[i] > 0.0f) {
      nonzero[nz_count++] = new_pitch[i];
    }
  }

  FP_TYPE average = (nz_count > 0) ? nonzero[nz_count / 2] : 0.0f;
  free(nonzero);

  for (int i = 0; i < new_nfrm; ++i) {
    new_pitch[i] = new_pitch[i] - average;
  }

  return new_pitch;
}

// การปรับการสั่นของระดับเสียง (Modulation) ตามค่าดั้งเดิม
FP_TYPE* apply_modulation(FP_TYPE* pitch, int mod, int nfrm) {
  FP_TYPE* modulated_pitch = malloc(sizeof(FP_TYPE) * nfrm);
  if (!modulated_pitch) return NULL;

  for (int i = 0; i < nfrm; ++i) {
    modulated_pitch[i] = pitch[i] * (mod / 100.0f);
  }
  return modulated_pitch;
}

// โครงสร้างรับค่าสถานะคำสั่งพิเศษ
typedef struct {
    int Mt;
    int t;
    int P;
    int g;
    int e;
} Flags;

int clamp(int val, int min, int max) {
    return val < min ? min : (val > max ? max : val);
}

// ประมวลผลสตริงการตั้งค่าคำสั่ง (Flag String Parsing)
void parse_flag_string(const char* str, Flags* flags_out) {
    int i = 0;
    flags_out->Mt = 0; 
    flags_out->t = 0;
    flags_out->P = 0;
    flags_out->g = 0;
    flags_out->e = 0; 

    while (str[i] != '\0') {
        if (strncmp(&str[i], "Mt", 2) == 0) {
            i += 2;
            flags_out->Mt = strtol(&str[i], (char**)&str, 10); 
            flags_out->Mt = clamp(flags_out->Mt, -100, 100);
            continue;
        } else if (str[i] == 't') {
            i++;
            flags_out->t = strtol(&str[i], (char**)&str, 10);
            flags_out->t = clamp(flags_out->t, -1200, 1200);
            continue;
        } else if (str[i] == 'P') {
            i++;
            flags_out->P = strtol(&str[i], (char**)&str, 10);
            flags_out->P = clamp(flags_out->P, 0, 100);
            continue;
        } else if (str[i] == 'g') {
            i++;
            flags_out->g = strtol(&str[i], (char**)&str, 10);
            flags_out->g = clamp(flags_out->g, -100, 100); 
            continue;
        } else if (str[i] == 'e') {
            i++;
            flags_out->e = 1; 
            continue;
        }
        i++;
    }
}

// ปรับระดับคลื่นเสียงให้สม่ำเสมอตามเป้าหมาย (Waveform Normalization)
void normalize_waveform(FP_TYPE *waveform, int length, FP_TYPE target_peak,
                        int P_flag) {
  if (P_flag <= 0)
    return;

  FP_TYPE peak = 0.0f;
  for (int i = 0; i < length; ++i) {
    FP_TYPE abs_val = fabsf(waveform[i]);
    if (abs_val > peak)
      peak = abs_val;
  }

  if (peak < 1e-9f)
    return; 

  FP_TYPE full_scale = target_peak / peak;
  FP_TYPE blend = P_flag / 100.0f;
  FP_TYPE scale = linterp(1.0f, full_scale, blend);

  for (int i = 0; i < length; ++i)
    waveform[i] *= scale;
}

int parse_tempo(const char *tempo_str) {
    if (tempo_str[0] == '!') {
        tempo_str++;  
    }
    return atoi(tempo_str);
}

typedef struct {
    char* input;  
    char* output; 
    float tone; 
    float velocity; 
    char* flags; 
    float offset; 
    float length; 
    float consonant; 
    float cutoff; 
    int volume; 
    int modulation; 
    int tempo; 
    char* pitch_curve; 
} resampler_data;

// ฟังก์ชันหลักในการปรับแต่งและสังเคราะห์เสียง (Resampling System)
int resample(resampler_data* data) {
  // สร้างและโหลดเส้นระดับเสียง
  double* f0_curve = malloc(sizeof(double) * 3000);
  if (!f0_curve) return 1;
  int pit_len = getF0Contour(data->pitch_curve, f0_curve);
  if (!pit_len) { free(f0_curve); return 1; }

  float velocity = (float)exp2(1 - data->velocity / 100.0f);
  Flags flags;
  parse_flag_string(data->flags, &flags);

  // เตรียมเส้นทางไฟล์ .llsm2 โดยใช้ Dynamic Allocation ป้องกัน Buffer Overflow
  size_t path_len = strlen(data->input) + 10;
  char* llsm_path = malloc(path_len);
  if (!llsm_path) { free(f0_curve); return 1; }
  
  snprintf(llsm_path, path_len, "%s", data->input);
  char* ext = strrchr(llsm_path, '.');
  if (ext) strcpy(ext, ".llsm2");  
  else strcat(llsm_path, ".llsm2");

  FILE* llsm_file = fopen(llsm_path, "rb");

  llsm_aoptions* opt_a = llsm_create_aoptions();
  llsm_soptions* opt_s = llsm_create_soptions(44100.0f);
  llsm_chunk* chunk = NULL;
  int nhop = 128;
  int fs = 0, nbit = 0, nx = 0;
  float* input = NULL;
  FP_TYPE* f0 = NULL;
  int nfrm = 0;

  if (llsm_file) {
    // หากพบไฟล์แคช ดำเนินการโหลดแทนการวิเคราะห์ใหม่
    fclose(llsm_file);
    printf("[ข้อมูล] กำลังโหลดผลการวิเคราะห์ LLSM จากแคช: %s\n", llsm_path);
    chunk = read_llsm(llsm_path, &nfrm, &fs, &nbit);
    
    if (!chunk) {
      printf("[ข้อผิดพลาด] ไม่สามารถอ่านไฟล์ .llsm2 ได้\n");
      free(llsm_path);
      free(f0_curve);
      return 1;
    }
  } else {
    // กรณีที่ไม่มีไฟล์แคช ดำเนินการวิเคราะห์เสียง (Audio Analysis)
    printf("[ขั้นตอน] กำลังอ่านไฟล์ WAV ต้นฉบับ: %s\n", data->input);
    input = wavread(data->input, &fs, &nbit, &nx);
    if (!input) { free(llsm_path); free(f0_curve); return 1; }

    printf("[ขั้นตอน] กำลังประเมินระดับความถี่ (F0 Estimation)\n");
    pyin_config param = pyin_init(nhop);
    param.fmin = 50.0f;
    param.fmax = 800.0f;
    param.trange = 24;
    param.bias = 2;
    param.nf = ceil(fs * 0.025);
    f0 = pyin_analyze(param, input, nx, fs, &nfrm);
    if (!f0) { free(input); free(llsm_path); free(f0_curve); return 1; }

    opt_a->thop = (FP_TYPE)nhop / fs;
    opt_a->f0_refine = 1;
    opt_a->hm_method = LLSM_AOPTION_HMCZT;

    printf("[ขั้นตอน] กำลังดำเนินการวิเคราะห์เสียง (Audio Analysis)\n");
    chunk = llsm_analyze(opt_a, input, nx, fs, f0, nfrm, NULL);
    if (!chunk) { free(input); free(f0); free(llsm_path); free(f0_curve); return 1; }

    printf("[ข้อมูล] กำลังบันทึกผลการวิเคราะห์ลงในแคช: %s\n", llsm_path);
    if (save_llsm(chunk, llsm_path, opt_a, &fs, &nbit) != 0) {
      printf("[ข้อผิดพลาด] ไม่สามารถบันทึกไฟล์ .llsm2 ได้\n");
    }

    free(input);
    free(f0);
  }
  printf("[ขั้นตอน] การซิงโครไนซ์เฟสและการยืดขยายเวลา (Phase Sync/Stretching)\n");
  
  // คำนวณเฟรมเริ่มต้นและสิ้นสุด
  int start_frame = (int)round((data->offset / 1000.0) * fs / nhop);
  int end_frame;
  if (data->cutoff < 0) {
      end_frame = (int)round(((data->offset + fabs(data->cutoff)) / 1000.0) * fs / nhop);
  } else {
      end_frame = nfrm - (int)round((data->cutoff / 1000.0) * fs / nhop);
  }
  if (start_frame < 0) start_frame = 0;
  if (end_frame > nfrm) end_frame = nfrm;
  if (end_frame <= start_frame) end_frame = start_frame + 1;
  
  int consonant_frames = (int)round((data->consonant / 1000.0) * fs / nhop);
  if (consonant_frames > end_frame - start_frame)
      consonant_frames = end_frame - start_frame;
  int sample_frames = end_frame - start_frame;
  int total_frames = (int)round((data->length / 1000.0) * fs / nhop);
  if (total_frames < consonant_frames) total_frames = consonant_frames + 1;
  
  float* f0_array = malloc(sizeof(float) * total_frames);
  if (!f0_array) {
    free(llsm_path);
    free(f0_curve);
    if (chunk) llsm_delete_chunk(chunk);
    if (opt_a) llsm_delete_aoptions(opt_a);
    if (opt_s) llsm_delete_soptions(opt_s);
    return 1;
  }

  FP_TYPE* temp_pitch = interp_pitch_to_pitd(get_pitch_from_area(chunk, start_frame, end_frame), sample_frames, total_frames);
  FP_TYPE* mod_pitch = apply_modulation(temp_pitch, data->modulation, total_frames);
  free(temp_pitch); // ทำการคืนค่าหน่วยความจำหลังใช้งานเสร็จ
  
  convert_cents_to_hz_offset(f0_curve, pit_len, total_frames, nhop, fs, data->tempo, f0_array);
  printf("[ข้อมูล] ความยาวพิทช์ (pit_len): %d, จำนวนเฟรมทั้งหมด: %d\n", pit_len, total_frames);
  
  for (int i = 0; i < total_frames; ++i) {
    f0_array[i] *= pow(2.0f, (double)flags.t/120);
  }
  
  llsm_container* conf_new = llsm_copy_container(chunk -> conf);
  llsm_container_attach(conf_new, LLSM_CONF_NFRM,
    llsm_create_int(total_frames), llsm_delete_int, llsm_copy_int);
  llsm_chunk* chunk_new = llsm_create_chunk(conf_new, 1);
  llsm_delete_container(conf_new);
  int no_stretch = 0;
  
  if (total_frames <= sample_frames) {
    for (int i = 0; i < total_frames; i++) {
      chunk_new->frames[i] = llsm_copy_container(chunk->frames[start_frame + i]);
    }
    no_stretch = 1;
  } else {
    for (int i = 0; i < sample_frames; i++) {
      chunk_new->frames[i] = llsm_copy_container(chunk->frames[start_frame + i]);
    }
  }
  llsm_chunk_tolayer1(chunk_new, 2048);
  llsm_chunk_phasepropagate(chunk_new, -1);
  printf("[ข้อมูล] จำนวนเฟรมสุดท้าย (nfrm): %d\n", total_frames);
  
  int frames_for_velocity = sample_frames;
  if (frames_for_velocity > total_frames)
      frames_for_velocity = total_frames;

  if (data->velocity != 100.0f) {
      apply_velocity(chunk_new, velocity, &consonant_frames, frames_for_velocity);
  }
  
  int vowel_sample_frames = sample_frames - consonant_frames;
  int vowel_total_frames  = total_frames - consonant_frames;

  if (vowel_sample_frames <= 0 || vowel_total_frames <= 0 || vowel_sample_frames >= vowel_total_frames) {
    no_stretch = 1;
  } else {
    no_stretch = 0;
  }
  
  if (no_stretch == 0) {
    for (int i = consonant_frames; i < total_frames; i++) {
      FP_TYPE mapped = (FP_TYPE)(i - consonant_frames) * vowel_sample_frames / vowel_total_frames;
      int base = consonant_frames + (int)mapped;
      FP_TYPE ratio = mapped - (int)mapped;
      int residx = base + rand() % 5 - 2;
      residx = max(consonant_frames, min(consonant_frames + vowel_sample_frames - 1, residx));
      base = min(base, consonant_frames + vowel_sample_frames - 2);
      chunk_new->frames[i] = llsm_copy_container(chunk_new->frames[base]);
      interp_llsm_frame(
        chunk_new->frames[i], chunk_new->frames[base + 1], ratio);
      FP_TYPE* resvec = llsm_container_get(chunk_new->frames[residx],
        LLSM_FRAME_PSDRES);
      if (resvec != NULL) {
        llsm_container_attach(chunk_new->frames[i], LLSM_FRAME_PSDRES,
          llsm_copy_fparray(resvec), llsm_delete_fparray, llsm_copy_fparray);
      }
    }
  }
  
  // กำหนดระดับเสียงไปตามโน้ตเป้าหมาย
  for(int i = 0; i < total_frames; i ++) {
    llsm_container_attach(chunk_new->frames[i], LLSM_FRAME_HM, NULL, NULL, NULL);
    FP_TYPE* f0_i = llsm_container_get(chunk_new->frames[i], LLSM_FRAME_F0);
    FP_TYPE old_f0 = f0_i[0];

    if (f0_i[0] == 0.00f) {
      continue;
    }
    else {
      f0_i[0] = data->tone * (1.0 + f0_array[i]);
      f0_i[0] += mod_pitch[i];
      if (f0_i[0] < 20.0f && f0_i[0] != 0.0f) f0_i[0] = 20.0f; // รักษาค่า F0 ขั้นต่ำ
    }
    // ชดเชยการสูญเสียระดับเสียง (Amplitude Gain)
    FP_TYPE* vt_magn = llsm_container_get(chunk_new->frames[i],
      LLSM_FRAME_VTMAGN);
    if(vt_magn != NULL) {
      int nspec = llsm_fparray_length(vt_magn);
      for(int j = 0; j < nspec; j ++)
        vt_magn[j] -= 20.0 * log10(f0_i[0] / old_f0);
    }
  }

  llsm_chunk_phasepropagate(chunk_new, 1);
  llsm_chunk_tolayer0(chunk_new);
  apply_tension(chunk_new, flags.Mt); 
  printf("[ขั้นตอน] กำลังดำเนินการสังเคราะห์เสียง (Synthesis)\n");
  
  llsm_output* out = llsm_synthesize(opt_s, chunk_new);

  if (!out || !out->y) {
      printf("[ข้อผิดพลาด] การสังเคราะห์ล้มเหลว (Synthesis Failed)\n");
      // จัดการคืนค่าหน่วยความจำกรณีล้มเหลว
      free(f0_array);
      free(mod_pitch);
      free(llsm_path);
      free(f0_curve);
      return 1;
  }

  normalize_waveform(out->y, out->ny, 0.60f, flags.P); 

  float scale = data->volume / 100.0f;
  for (int i = 0; i < out->ny; ++i)
    out->y[i] *= scale;
  
  wavwrite(out->y, out->ny, fs, nbit, data->output);
  
  // จัดการทำความสะอาดและคืนพื้นที่หน่วยความจำ
  llsm_delete_output(out);
  llsm_delete_chunk(chunk);
  llsm_delete_chunk(chunk_new);
  llsm_delete_aoptions(opt_a);
  llsm_delete_soptions(opt_s);
  free(f0_array);
  free(mod_pitch);
  free(llsm_path);
  free(f0_curve);
  
  printf("[สำเร็จ] การประมวลผลเสร็จสิ้น\n");
  return 0;
}

int main(int argc, char* argv[]) {
    printf("Moresampler2 เวอร์ชัน %s\n", version);
    if (argc == 2) { 
        printf("[ระบบ] ขณะนี้ยังไม่รองรับระบบการสร้างฉลากอัตโนมัติ (Autolabeling)\n");
        return 0;
    }
    if (argc < 2) {
        printf("[ข้อผิดพลาด] Moresampler ถูกออกแบบมาเพื่อใช้งานภายใน UTAU หรือ OpenUtau เท่านั้น\n");
        return 1;
    }
    if (argc == 14) { // เข้าสู่โหมด Resampler ตามพารามิเตอร์ของ UTAU
        resampler_data data;
        data.input = argv[1];
        data.output = argv[2];
        data.tone = note_to_frequency(argv[3]); 
        data.velocity = atof(argv[4]);
        data.flags = argv[5]; 
        data.offset = atof(argv[6]);
        data.length = atof(argv[7]);
        data.consonant = atof(argv[8]);
        data.cutoff = atof(argv[9]);
        data.volume = atoi(argv[10]);
        data.modulation = atoi(argv[11]);
        data.tempo = parse_tempo(argv[12]); 
        data.pitch_curve = argv[13]; 
        return resample(&data);
    }
    printf("[ข้อผิดพลาด] พารามิเตอร์ไม่ถูกต้อง คาดหวังการรับค่า 14 ส่วน แต่ได้รับมา %d ส่วน\n", argc);
    return 0;
}
