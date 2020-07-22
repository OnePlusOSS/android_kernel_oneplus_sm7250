#ifndef __INCLUDE_PCCORE__
#define __INCLUDE_PCCORE__

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define PCC_TAG "pccore:"

#define PCC_PARAMS 4
#define NR_CLU 3

#define pcc_logv(fmt...) \
	do { \
		if (pcclog_lv < 1) \
			pr_info(PCC_TAG fmt); \
	} while (0)

#define pcc_logi(fmt...) \
	do { \
		if (pcclog_lv < 2) \
			pr_info(PCC_TAG fmt); \
	} while (0)

#define pcc_logw(fmt...) \
	do { \
		if (pcclog_lv < 3) \
			pr_warn(PCC_TAG fmt); \
	} while (0)

#define pcc_loge(fmt...) pr_err(PCC_TAG fmt)
#define pcc_logd(fmt...) pr_debug(PCC_TAG fmt)
// 2020/04/04 to accommodate the SM7250_AC
static unsigned int cluster_pd[NR_CLU] = {9, 8, 8};
static unsigned int cpufreq_pd_0[9] = {
	0,//300000
	0,//576000
	0,//614400
	1,//864000
	2,//1075200
	3,//1363200
	4,//1516800
	5,//1651200
	6,//1804800
};

// 2020/04/04 to accommodate the SM7250_AC
static unsigned int cpufreq_pd_1[8] = {
	0,//652800
	1,//940800
	2,//1152000
	3,//1478400
	4,//1728000
	5,//1900800
	6,//2092800
	7,//2208000
};

// 2020/04/04 to accommodate the SM7250_AC
static unsigned int cpufreq_pd_2[8] = {
	0,//806400
	1,//1094400
	2,//1401600
	3,//1766400
	4,//1996800
	5,//2188800
	6,//2304000
	7,//2400000
};

#endif // __INCLUDE_PCCORE__
