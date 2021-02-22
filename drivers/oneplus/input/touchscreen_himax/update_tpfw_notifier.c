// SPDX-License-Identifier: GPL-2.0
/*  Himax Android Driver Sample Code for HX83112 chipset
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/notifier.h>
#include <linux/update_tpfw_notifier.h>

BLOCKING_NOTIFIER_HEAD(tpfw_notifier_list);

int update_tpfw_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&tpfw_notifier_list, nb);
}
EXPORT_SYMBOL(update_tpfw_register_client);

int update_tpfw_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&tpfw_notifier_list, nb);
}
EXPORT_SYMBOL(update_tpfw_unregister_client);


int update_tpfw_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&tpfw_notifier_list, val, v);

}
EXPORT_SYMBOL_GPL(update_tpfw_notifier_call_chain);


