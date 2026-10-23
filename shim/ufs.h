#pragma once
void test_ufs(void);

/* Read the kernel's persistent console ring (patch 0047) left by the
   previous boot from userdata and print its tail on the console. */
void ufs_log_dump(void);
