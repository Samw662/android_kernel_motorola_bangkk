// SPDX-License-Identifier: GPL-2.0
/*
 * WearyStars - kernel identity easter egg.
 * A /proc/wearystars panel showing kernel status live (freqs, governor,
 * boost), plus a boot banner. Pure userland-visible, no side effects.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>

static const char * const weary_banner[] = {
	"__        __   _                          _____ _                 _ ",
	"\\ \\      / /__| | ___ ___  _ __ ___   __  |___  | |__   ___  _ __| |",
	" \\ \\ /\\ / / _ \\ |/ __/ _ \\| '_ ` _ \\ / _` |   / /| '_ \\ / _ \\| '__| __|",
	"  \\ V  V /  __/ | (_| (_) | | | | | | (_| |  / / | | | | (_) | |  | |_",
	"   \\_/\\_/ \\___|_|\\___\\___/|_| |_| |_|\\__,_| /_/  |_| |_|\\___/|_|   \\__|",
	"",
	"kernel . frankenstein . builtin-max",
};

static int weary_show(struct seq_file *m, void *v)
{
	int cpu;

	for (cpu = 0; cpu < ARRAY_SIZE(weary_banner); cpu++)
		seq_printf(m, "%s\n", weary_banner[cpu]);

	seq_printf(m, "\n%s / %s / %d cores\n",
		   utsname()->release, utsname()->machine,
		   num_online_cpus());
	seq_puts(m, "---- live status (touch the screen!) ----\n");

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		unsigned int cur = cpufreq_quick_get(cpu);

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		seq_printf(m, "cpu%d: %d kHz  gov=%s\n",
			   cpu, cur ? cur : policy->cur,
			   policy->governor ? policy->governor->name : "?");
		cpufreq_cpu_put(policy);
	}
	return 0;
}

static int weary_open(struct inode *inode, struct file *file)
{
	return single_open(file, weary_show, NULL);
}

static const struct file_operations weary_fops = {
	.open		= weary_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init wearystars_init(void)
{
	int i;

	proc_create("wearystars", 0444, NULL, &weary_fops);

	for (i = 0; i < ARRAY_SIZE(weary_banner); i++)
		pr_info("WearyStars: %s\n", weary_banner[i]);
	pr_info("WearyStars: %s/%s up, %d cores online\n",
		utsname()->release, utsname()->machine, num_online_cpus());

	return 0;
}
late_initcall(wearystars_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("WearyStars kernel identity easter egg");
