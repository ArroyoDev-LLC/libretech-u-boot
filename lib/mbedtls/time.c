// SPDX-License-Identifier: GPL-2.0+
/*
 * mbedTLS's clock, which X.509 validity dates are checked against.
 *
 * It never reads earlier than this build's UTC time, U_BOOT_EPOCH. With no
 * RTC, an RTC that was never set, or one set back (sntp failed or answered
 * wrong, rtc_emul_epoch, `date`), a certificate that had expired by the time
 * this bootloader was built is still refused, so the clock cannot roll TLS
 * back. An RTC ahead of the build moves it forward.
 */

#include <dm.h>
#include <rtc.h>
#include <time.h>
#include <timestamp.h>
#include <linux/time.h>
#include <mbedtls/platform_time.h>
#include <mbedtls/platform_util.h>

#if defined(MBEDTLS_HAVE_TIME_DATE)

time_t u_boot_mbedtls_time(time_t *t)
{
	time_t now = U_BOOT_EPOCH;
#if CONFIG_IS_ENABLED(DM_RTC)
	struct udevice *dev;
	struct rtc_time tm;

	if (!uclass_get_device(UCLASS_RTC, 0, &dev) && !dm_rtc_get(dev, &tm)) {
		time_t rtc = rtc_mktime(&tm);

		if (rtc > now)
			now = rtc;
	}
#endif
	if (t)
		*t = now;
	return now;
}

struct tm *mbedtls_platform_gmtime_r(const mbedtls_time_t *tt,
				     struct tm *tm_buf)
{
	struct rtc_time rt;

	if (*tt < 0)
		return NULL;
	rtc_to_tm(*tt, &rt);
	memset(tm_buf, 0, sizeof(*tm_buf));
	tm_buf->tm_sec = rt.tm_sec;
	tm_buf->tm_min = rt.tm_min;
	tm_buf->tm_hour = rt.tm_hour;
	tm_buf->tm_mday = rt.tm_mday;
	tm_buf->tm_mon = rt.tm_mon - 1;		/* rtc_time: January = 1 */
	tm_buf->tm_year = rt.tm_year - 1900;	/* rtc_time: the full year */
	tm_buf->tm_wday = rt.tm_wday;
	return tm_buf;
}

mbedtls_ms_time_t mbedtls_ms_time(void)
{
	return get_timer(0);
}

#endif /* MBEDTLS_HAVE_TIME_DATE */
