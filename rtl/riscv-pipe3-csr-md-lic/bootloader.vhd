-- #################################################################################################
-- # bootloader.vhd - The bootloader ROM                                                           #
-- # ********************************************************************************************* #
-- # This file is part of the THUAS RISCV RV32 Project                                             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Jesse op den Brouw. All rights reserved.                                  #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # https:/github.com/jesseopdenbrouw/riscv-rv32                                                  #
-- #################################################################################################

-- This file contains the description of the bootloader ROM. The ROM
-- is placed in immutable onboard RAM blocks. A read takes two
-- clock cycles, for both instruction and data.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.processor_common.all;
use work.processor_common_rom.all;

entity bootloader is
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          I_pc : in data_type;
          I_memaddress : in data_type;
          I_memsize : in memsize_type;
          I_csboot : in std_logic;
          I_stall : in std_logic;
          O_instr : out data_type;
          O_dataout : out data_type;
          --
          O_instruction_misaligned_error : out std_logic;
          O_load_misaligned_error : out std_logic
         );
end entity bootloader;

architecture rtl of bootloader is

-- The bootloader ROM
signal bootrom : bootloader_type := (
           0 => x"97020000",
           1 => x"93820208",
           2 => x"73905230",
           3 => x"97010010",
           4 => x"9381417f",
           5 => x"17810010",
           6 => x"1301c1fe",
           7 => x"9387c187",
           8 => x"1387c187",
           9 => x"13060000",
          10 => x"63e4e700",
          11 => x"3386e740",
          12 => x"93050000",
          13 => x"1385c187",
          14 => x"ef000047",
          15 => x"37050020",
          16 => x"9387c187",
          17 => x"13070500",
          18 => x"13060000",
          19 => x"63e4e700",
          20 => x"3386e740",
          21 => x"b7150010",
          22 => x"9385c5e5",
          23 => x"13050500",
          24 => x"ef004042",
          25 => x"ef00101b",
          26 => x"b7050020",
          27 => x"13060000",
          28 => x"93850500",
          29 => x"13055000",
          30 => x"ef000047",
          31 => x"ef009015",
          32 => x"6f000000",
          33 => x"b70700f0",
          34 => x"03a54702",
          35 => x"13754500",
          36 => x"67800000",
          37 => x"370700f0",
          38 => x"83274702",
          39 => x"93f74700",
          40 => x"e38c07fe",
          41 => x"03258702",
          42 => x"1375f50f",
          43 => x"67800000",
          44 => x"130101fd",
          45 => x"23202103",
          46 => x"37190010",
          47 => x"23248102",
          48 => x"23229102",
          49 => x"232e3101",
          50 => x"232c4101",
          51 => x"232a5101",
          52 => x"23286101",
          53 => x"23267101",
          54 => x"23248101",
          55 => x"23261102",
          56 => x"93040500",
          57 => x"13040000",
          58 => x"130989b0",
          59 => x"930a5001",
          60 => x"938bf5ff",
          61 => x"130bf007",
          62 => x"130a2000",
          63 => x"93092001",
          64 => x"371c0010",
          65 => x"eff01ff9",
          66 => x"1377f50f",
          67 => x"63c4ea0a",
          68 => x"6354ea04",
          69 => x"9307d7ff",
          70 => x"63e0f904",
          71 => x"93972700",
          72 => x"b307f900",
          73 => x"83a70700",
          74 => x"67800700",
          75 => x"630a0400",
          76 => x"1304f4ff",
          77 => x"1305f007",
          78 => x"ef004011",
          79 => x"e31a04fe",
          80 => x"eff05ff5",
          81 => x"1377f50f",
          82 => x"13040000",
          83 => x"e3d2eafc",
          84 => x"9307f007",
          85 => x"630ef70c",
          86 => x"63547405",
          87 => x"9377f50f",
          88 => x"938607fe",
          89 => x"93f6f60f",
          90 => x"1306e005",
          91 => x"e36cd6f8",
          92 => x"b3868400",
          93 => x"13050700",
          94 => x"2380f600",
          95 => x"ef00000d",
          96 => x"eff05ff1",
          97 => x"1377f50f",
          98 => x"93075001",
          99 => x"13041400",
         100 => x"e3d0e7f8",
         101 => x"9307f007",
         102 => x"6302f702",
         103 => x"e34074fd",
         104 => x"13057000",
         105 => x"ef00800a",
         106 => x"eff0dfee",
         107 => x"1377f50f",
         108 => x"e3d0eaf6",
         109 => x"e31267fb",
         110 => x"630c0406",
         111 => x"1305f007",
         112 => x"ef00c008",
         113 => x"1304f4ff",
         114 => x"6ff0dff3",
         115 => x"b3848400",
         116 => x"37150010",
         117 => x"23800400",
         118 => x"1305c5b7",
         119 => x"ef000009",
         120 => x"8320c102",
         121 => x"13050400",
         122 => x"03248102",
         123 => x"83244102",
         124 => x"03290102",
         125 => x"8329c101",
         126 => x"032a8101",
         127 => x"832a4101",
         128 => x"032b0101",
         129 => x"832bc100",
         130 => x"032c8100",
         131 => x"13010103",
         132 => x"67800000",
         133 => x"13058cd4",
         134 => x"ef004005",
         135 => x"eff09fe7",
         136 => x"1377f50f",
         137 => x"13040000",
         138 => x"e3d4eaee",
         139 => x"6ff05ff2",
         140 => x"13057000",
         141 => x"ef008001",
         142 => x"6ff09ff0",
         143 => x"b70700f0",
         144 => x"23a6a702",
         145 => x"23a0b702",
         146 => x"67800000",
         147 => x"1375f50f",
         148 => x"b70700f0",
         149 => x"370700f0",
         150 => x"23a4a702",
         151 => x"83274702",
         152 => x"93f70701",
         153 => x"e38c07fe",
         154 => x"67800000",
         155 => x"630e0502",
         156 => x"130101ff",
         157 => x"23248100",
         158 => x"23261100",
         159 => x"13040500",
         160 => x"03450500",
         161 => x"630a0500",
         162 => x"13041400",
         163 => x"eff01ffc",
         164 => x"03450400",
         165 => x"e31a05fe",
         166 => x"8320c100",
         167 => x"03248100",
         168 => x"13010101",
         169 => x"67800000",
         170 => x"67800000",
         171 => x"130101fe",
         172 => x"232e1100",
         173 => x"232c8100",
         174 => x"232a9100",
         175 => x"23282101",
         176 => x"23263101",
         177 => x"23244101",
         178 => x"6358a008",
         179 => x"b7190010",
         180 => x"13090500",
         181 => x"93040000",
         182 => x"13040000",
         183 => x"938959d5",
         184 => x"130a1000",
         185 => x"6f000001",
         186 => x"3364c400",
         187 => x"93841400",
         188 => x"63029904",
         189 => x"eff01fda",
         190 => x"b387a900",
         191 => x"83c70700",
         192 => x"130605fd",
         193 => x"13144400",
         194 => x"13f74700",
         195 => x"93f64704",
         196 => x"e31c07fc",
         197 => x"93f73700",
         198 => x"e38a06fc",
         199 => x"63944701",
         200 => x"13050502",
         201 => x"130595fa",
         202 => x"93841400",
         203 => x"3364a400",
         204 => x"e31299fc",
         205 => x"8320c101",
         206 => x"13050400",
         207 => x"03248101",
         208 => x"83244101",
         209 => x"03290101",
         210 => x"8329c100",
         211 => x"032a8100",
         212 => x"13010102",
         213 => x"67800000",
         214 => x"13040000",
         215 => x"6ff09ffd",
         216 => x"83470500",
         217 => x"37160010",
         218 => x"130656d5",
         219 => x"3307f600",
         220 => x"03470700",
         221 => x"93060500",
         222 => x"13758700",
         223 => x"630e0500",
         224 => x"83c71600",
         225 => x"93861600",
         226 => x"3307f600",
         227 => x"03470700",
         228 => x"13758700",
         229 => x"e31605fe",
         230 => x"13754704",
         231 => x"630a0506",
         232 => x"13050000",
         233 => x"13031000",
         234 => x"6f000002",
         235 => x"83c71600",
         236 => x"33e5a800",
         237 => x"93861600",
         238 => x"3307f600",
         239 => x"03470700",
         240 => x"13784704",
         241 => x"63000804",
         242 => x"13784700",
         243 => x"938807fd",
         244 => x"13773700",
         245 => x"13154500",
         246 => x"e31a08fc",
         247 => x"63146700",
         248 => x"93870702",
         249 => x"938797fa",
         250 => x"33e5a700",
         251 => x"83c71600",
         252 => x"93861600",
         253 => x"3307f600",
         254 => x"03470700",
         255 => x"13784704",
         256 => x"e31408fc",
         257 => x"63840500",
         258 => x"23a0d500",
         259 => x"67800000",
         260 => x"13050000",
         261 => x"6ff01fff",
         262 => x"130101fe",
         263 => x"232e1100",
         264 => x"23220100",
         265 => x"23240100",
         266 => x"23060100",
         267 => x"9387f5ff",
         268 => x"13077000",
         269 => x"6376f700",
         270 => x"93077000",
         271 => x"93058000",
         272 => x"13074100",
         273 => x"b307f700",
         274 => x"b385b740",
         275 => x"13069003",
         276 => x"9376f500",
         277 => x"13870603",
         278 => x"6374e600",
         279 => x"13877605",
         280 => x"2380e700",
         281 => x"9387f7ff",
         282 => x"13554500",
         283 => x"e392f5fe",
         284 => x"13054100",
         285 => x"eff09fdf",
         286 => x"8320c101",
         287 => x"13010102",
         288 => x"67800000",
         289 => x"13030500",
         290 => x"630e0600",
         291 => x"83830500",
         292 => x"23007300",
         293 => x"1306f6ff",
         294 => x"13031300",
         295 => x"93851500",
         296 => x"e31606fe",
         297 => x"67800000",
         298 => x"13030500",
         299 => x"630a0600",
         300 => x"2300b300",
         301 => x"1306f6ff",
         302 => x"13031300",
         303 => x"e31a06fe",
         304 => x"67800000",
         305 => x"03460500",
         306 => x"83c60500",
         307 => x"13051500",
         308 => x"93851500",
         309 => x"6314d600",
         310 => x"e31606fe",
         311 => x"3305d640",
         312 => x"67800000",
         313 => x"6f000000",
         314 => x"130101f8",
         315 => x"232e1106",
         316 => x"232c8106",
         317 => x"232a9106",
         318 => x"23282107",
         319 => x"23263107",
         320 => x"23244107",
         321 => x"23225107",
         322 => x"23206107",
         323 => x"232e7105",
         324 => x"232c8105",
         325 => x"232a9105",
         326 => x"2328a105",
         327 => x"2326b105",
         328 => x"732510fc",
         329 => x"63160500",
         330 => x"37e5f505",
         331 => x"13050510",
         332 => x"b7c70100",
         333 => x"93870720",
         334 => x"3355f502",
         335 => x"93050000",
         336 => x"b7091000",
         337 => x"3709a000",
         338 => x"93041000",
         339 => x"9389f9ff",
         340 => x"370a00f0",
         341 => x"13091900",
         342 => x"1305f5ff",
         343 => x"eff01fce",
         344 => x"37150010",
         345 => x"130545b5",
         346 => x"eff05fd0",
         347 => x"b70700f0",
         348 => x"1307f03f",
         349 => x"23a2e700",
         350 => x"b3f73401",
         351 => x"639c0700",
         352 => x"1305a002",
         353 => x"eff09fcc",
         354 => x"83274a00",
         355 => x"93d71700",
         356 => x"2322fa00",
         357 => x"eff01faf",
         358 => x"13040500",
         359 => x"631e050c",
         360 => x"93841400",
         361 => x"e39a24fd",
         362 => x"b70700f0",
         363 => x"23a20700",
         364 => x"631a0400",
         365 => x"93050000",
         366 => x"13050000",
         367 => x"eff01fc8",
         368 => x"e7000400",
         369 => x"eff01fad",
         370 => x"93071002",
         371 => x"93040000",
         372 => x"631cf51c",
         373 => x"37140010",
         374 => x"130584b7",
         375 => x"eff01fc9",
         376 => x"370900f0",
         377 => x"130a3005",
         378 => x"930aa004",
         379 => x"130b3002",
         380 => x"93092000",
         381 => x"930ba000",
         382 => x"83274900",
         383 => x"93c71700",
         384 => x"2322f900",
         385 => x"eff01fa9",
         386 => x"1375f50f",
         387 => x"63184517",
         388 => x"eff05fa8",
         389 => x"137cf50f",
         390 => x"9307fcfc",
         391 => x"93f7f70f",
         392 => x"63e0f910",
         393 => x"93071003",
         394 => x"631cfc04",
         395 => x"13052000",
         396 => x"eff0dfc7",
         397 => x"930cd5ff",
         398 => x"13054000",
         399 => x"eff01fc7",
         400 => x"370d01ff",
         401 => x"b70d0001",
         402 => x"130c0500",
         403 => x"b38cac00",
         404 => x"130dfdff",
         405 => x"938dfdff",
         406 => x"631a9c05",
         407 => x"130ca000",
         408 => x"eff05fa3",
         409 => x"1375f50f",
         410 => x"e31c85ff",
         411 => x"130584b7",
         412 => x"eff0dfbf",
         413 => x"6ff05ff8",
         414 => x"13041000",
         415 => x"6ff0dff2",
         416 => x"93072003",
         417 => x"13052000",
         418 => x"631afc00",
         419 => x"eff01fc2",
         420 => x"930cc5ff",
         421 => x"13056000",
         422 => x"6ff05ffa",
         423 => x"eff01fc1",
         424 => x"930cb5ff",
         425 => x"13058000",
         426 => x"6ff05ff9",
         427 => x"1376ccff",
         428 => x"13052000",
         429 => x"2326c100",
         430 => x"eff05fbf",
         431 => x"0326c100",
         432 => x"93070500",
         433 => x"b706ffff",
         434 => x"13753c00",
         435 => x"03270600",
         436 => x"93053000",
         437 => x"13081000",
         438 => x"9386f60f",
         439 => x"63063503",
         440 => x"630ab502",
         441 => x"630c0501",
         442 => x"137707f0",
         443 => x"b3e7e700",
         444 => x"2320f600",
         445 => x"130c1c00",
         446 => x"6ff01ff6",
         447 => x"3377d700",
         448 => x"93978700",
         449 => x"6ff09ffe",
         450 => x"3377a701",
         451 => x"93970701",
         452 => x"6ff0dffd",
         453 => x"3377b701",
         454 => x"93978701",
         455 => x"6ff01ffd",
         456 => x"93079cfc",
         457 => x"93f7f70f",
         458 => x"63e2f904",
         459 => x"13052000",
         460 => x"eff0dfb7",
         461 => x"93077003",
         462 => x"13058000",
         463 => x"630afc00",
         464 => x"93078003",
         465 => x"13056000",
         466 => x"6304fc00",
         467 => x"13054000",
         468 => x"eff0dfb5",
         469 => x"93040500",
         470 => x"130ca000",
         471 => x"eff09f93",
         472 => x"1375f50f",
         473 => x"e31c85ff",
         474 => x"6ff05ff0",
         475 => x"eff09f92",
         476 => x"1375f50f",
         477 => x"e31c75ff",
         478 => x"6ff05fef",
         479 => x"631a5509",
         480 => x"130584b7",
         481 => x"eff09fae",
         482 => x"93050000",
         483 => x"13050000",
         484 => x"eff0dfaa",
         485 => x"23220900",
         486 => x"e7800400",
         487 => x"b70700f0",
         488 => x"1307a00a",
         489 => x"23a2e700",
         490 => x"37190010",
         491 => x"1305c9b7",
         492 => x"b7190010",
         493 => x"eff09fab",
         494 => x"13040000",
         495 => x"371b0010",
         496 => x"b71b0010",
         497 => x"938959d5",
         498 => x"b7170010",
         499 => x"138507b8",
         500 => x"eff0dfa9",
         501 => x"93059002",
         502 => x"13054101",
         503 => x"eff05f8d",
         504 => x"13054101",
         505 => x"ef00c02c",
         506 => x"b7170010",
         507 => x"130a0500",
         508 => x"938547b8",
         509 => x"13054101",
         510 => x"eff0dfcc",
         511 => x"631e0500",
         512 => x"37150010",
         513 => x"130585b8",
         514 => x"eff05fa6",
         515 => x"6f004003",
         516 => x"e31e65e5",
         517 => x"6ff09ff8",
         518 => x"b7170010",
         519 => x"938547c7",
         520 => x"13054101",
         521 => x"eff01fca",
         522 => x"63100502",
         523 => x"93050000",
         524 => x"eff0dfa0",
         525 => x"b70700f0",
         526 => x"23a20700",
         527 => x"e7800400",
         528 => x"e3040af8",
         529 => x"6f004018",
         530 => x"b7170010",
         531 => x"13063000",
         532 => x"938587c7",
         533 => x"13054101",
         534 => x"ef004027",
         535 => x"63100504",
         536 => x"93050000",
         537 => x"13057101",
         538 => x"eff09faf",
         539 => x"93773500",
         540 => x"13040500",
         541 => x"63940706",
         542 => x"93058000",
         543 => x"eff0dfb9",
         544 => x"37150010",
         545 => x"1305c5c7",
         546 => x"eff05f9e",
         547 => x"03250400",
         548 => x"93058000",
         549 => x"eff05fb8",
         550 => x"6ff09ffa",
         551 => x"13063000",
         552 => x"93058bc9",
         553 => x"13054101",
         554 => x"ef004022",
         555 => x"631e0502",
         556 => x"93050101",
         557 => x"13057101",
         558 => x"eff09faa",
         559 => x"93773500",
         560 => x"13040500",
         561 => x"639c0700",
         562 => x"03250101",
         563 => x"93050000",
         564 => x"eff01fa9",
         565 => x"2320a400",
         566 => x"6ff09ff6",
         567 => x"37150010",
         568 => x"130505c8",
         569 => x"6ff05ff2",
         570 => x"13063000",
         571 => x"9385cbc9",
         572 => x"13054101",
         573 => x"ef00801d",
         574 => x"83474101",
         575 => x"1307e006",
         576 => x"630c0508",
         577 => x"639ae70a",
         578 => x"93773400",
         579 => x"e39807fc",
         580 => x"130c0404",
         581 => x"b71c0010",
         582 => x"371d0010",
         583 => x"930d80ff",
         584 => x"93058000",
         585 => x"13050400",
         586 => x"eff01faf",
         587 => x"1385ccc7",
         588 => x"eff0df93",
         589 => x"83270400",
         590 => x"93058000",
         591 => x"130a8001",
         592 => x"13850700",
         593 => x"2326f100",
         594 => x"eff01fad",
         595 => x"13050dca",
         596 => x"eff0df91",
         597 => x"b70a00ff",
         598 => x"8327c100",
         599 => x"33f55701",
         600 => x"33554501",
         601 => x"b3063501",
         602 => x"83c60600",
         603 => x"93f67609",
         604 => x"63800604",
         605 => x"130a8aff",
         606 => x"eff05f8d",
         607 => x"93da8a00",
         608 => x"e31cbafd",
         609 => x"13044400",
         610 => x"1305c9b7",
         611 => x"eff01f8e",
         612 => x"e31884f9",
         613 => x"6ff05fe3",
         614 => x"e388e7f6",
         615 => x"93050000",
         616 => x"13057101",
         617 => x"eff0df9b",
         618 => x"13040500",
         619 => x"6ff0dff5",
         620 => x"1305e002",
         621 => x"6ff01ffc",
         622 => x"e3080ae0",
         623 => x"37150010",
         624 => x"130545ca",
         625 => x"eff09f8a",
         626 => x"1305c9b7",
         627 => x"eff01f8a",
         628 => x"6ff09fdf",
         629 => x"130101ff",
         630 => x"23248100",
         631 => x"23261100",
         632 => x"93070000",
         633 => x"13040500",
         634 => x"63880700",
         635 => x"93050000",
         636 => x"97000000",
         637 => x"e7000000",
         638 => x"b7170010",
         639 => x"03a587e5",
         640 => x"83278502",
         641 => x"63840700",
         642 => x"e7800700",
         643 => x"13050400",
         644 => x"eff05fad",
         645 => x"130101ff",
         646 => x"23248100",
         647 => x"23229100",
         648 => x"37140010",
         649 => x"b7140010",
         650 => x"9387c4e5",
         651 => x"1304c4e5",
         652 => x"3304f440",
         653 => x"23202101",
         654 => x"23261100",
         655 => x"13542440",
         656 => x"9384c4e5",
         657 => x"13090000",
         658 => x"63108904",
         659 => x"b7140010",
         660 => x"37140010",
         661 => x"9387c4e5",
         662 => x"1304c4e5",
         663 => x"3304f440",
         664 => x"13542440",
         665 => x"9384c4e5",
         666 => x"13090000",
         667 => x"63188902",
         668 => x"8320c100",
         669 => x"03248100",
         670 => x"83244100",
         671 => x"03290100",
         672 => x"13010101",
         673 => x"67800000",
         674 => x"83a70400",
         675 => x"13091900",
         676 => x"93844400",
         677 => x"e7800700",
         678 => x"6ff01ffb",
         679 => x"83a70400",
         680 => x"13091900",
         681 => x"93844400",
         682 => x"e7800700",
         683 => x"6ff01ffc",
         684 => x"93070500",
         685 => x"03c70700",
         686 => x"93871700",
         687 => x"e31c07fe",
         688 => x"3385a740",
         689 => x"1305f5ff",
         690 => x"67800000",
         691 => x"630a0602",
         692 => x"1306f6ff",
         693 => x"13070000",
         694 => x"b307e500",
         695 => x"b386e500",
         696 => x"83c70700",
         697 => x"83c60600",
         698 => x"6398d700",
         699 => x"6306c700",
         700 => x"13071700",
         701 => x"e39207fe",
         702 => x"3385d740",
         703 => x"67800000",
         704 => x"13050000",
         705 => x"67800000",
         706 => x"14020010",
         707 => x"58010010",
         708 => x"58010010",
         709 => x"58010010",
         710 => x"58010010",
         711 => x"b8010010",
         712 => x"58010010",
         713 => x"cc010010",
         714 => x"58010010",
         715 => x"58010010",
         716 => x"cc010010",
         717 => x"58010010",
         718 => x"58010010",
         719 => x"58010010",
         720 => x"58010010",
         721 => x"58010010",
         722 => x"58010010",
         723 => x"58010010",
         724 => x"2c010010",
         725 => x"0d0a5448",
         726 => x"55415320",
         727 => x"52495343",
         728 => x"2d562042",
         729 => x"6f6f746c",
         730 => x"6f616465",
         731 => x"72207630",
         732 => x"2e330d0a",
         733 => x"00000000",
         734 => x"3f0a0000",
         735 => x"0d0a0000",
         736 => x"3e200000",
         737 => x"68000000",
         738 => x"48656c70",
         739 => x"3a0d0a20",
         740 => x"68202020",
         741 => x"20202020",
         742 => x"20202020",
         743 => x"20202020",
         744 => x"202d2074",
         745 => x"68697320",
         746 => x"68656c70",
         747 => x"0d0a2072",
         748 => x"20202020",
         749 => x"20202020",
         750 => x"20202020",
         751 => x"20202020",
         752 => x"2d207275",
         753 => x"6e206170",
         754 => x"706c6963",
         755 => x"6174696f",
         756 => x"6e0d0a20",
         757 => x"7277203c",
         758 => x"61646472",
         759 => x"3e202020",
         760 => x"20202020",
         761 => x"202d2072",
         762 => x"65616420",
         763 => x"776f7264",
         764 => x"2066726f",
         765 => x"6d206164",
         766 => x"64720d0a",
         767 => x"20777720",
         768 => x"3c616464",
         769 => x"723e203c",
         770 => x"64617461",
         771 => x"3e202d20",
         772 => x"77726974",
         773 => x"6520776f",
         774 => x"72642064",
         775 => x"61746120",
         776 => x"61742061",
         777 => x"6464720d",
         778 => x"0a206477",
         779 => x"203c6164",
         780 => x"64723e20",
         781 => x"20202020",
         782 => x"2020202d",
         783 => x"2064756d",
         784 => x"70203136",
         785 => x"20776f72",
         786 => x"64730d0a",
         787 => x"206e2020",
         788 => x"20202020",
         789 => x"20202020",
         790 => x"20202020",
         791 => x"20202d20",
         792 => x"64756d70",
         793 => x"206e6578",
         794 => x"74203136",
         795 => x"20776f72",
         796 => x"64730000",
         797 => x"72000000",
         798 => x"72772000",
         799 => x"3a200000",
         800 => x"4e6f7420",
         801 => x"6f6e2034",
         802 => x"2d627974",
         803 => x"6520626f",
         804 => x"756e6461",
         805 => x"72792100",
         806 => x"77772000",
         807 => x"64772000",
         808 => x"20200000",
         809 => x"3f3f0000",
         810 => x"626f6f74",
         811 => x"6c6f6164",
         812 => x"65720000",
         813 => x"54485541",
         814 => x"53205249",
         815 => x"53432d56",
         816 => x"20525633",
         817 => x"32494d20",
         818 => x"62617265",
         819 => x"206d6574",
         820 => x"616c2070",
         821 => x"726f6365",
         822 => x"73736f72",
         823 => x"00000000",
         824 => x"54686520",
         825 => x"48616775",
         826 => x"6520556e",
         827 => x"69766572",
         828 => x"73697479",
         829 => x"206f6620",
         830 => x"4170706c",
         831 => x"69656420",
         832 => x"53636965",
         833 => x"6e636573",
         834 => x"00000000",
         835 => x"44657061",
         836 => x"72746d65",
         837 => x"6e74206f",
         838 => x"6620456c",
         839 => x"65637472",
         840 => x"6963616c",
         841 => x"20456e67",
         842 => x"696e6565",
         843 => x"72696e67",
         844 => x"00000000",
         845 => x"4a2e452e",
         846 => x"4a2e206f",
         847 => x"70206465",
         848 => x"6e204272",
         849 => x"6f757700",
         850 => x"3c627265",
         851 => x"616b3e0d",
         852 => x"0a000000",
         853 => x"00202020",
         854 => x"20202020",
         855 => x"20202828",
         856 => x"28282820",
         857 => x"20202020",
         858 => x"20202020",
         859 => x"20202020",
         860 => x"20202020",
         861 => x"20881010",
         862 => x"10101010",
         863 => x"10101010",
         864 => x"10101010",
         865 => x"10040404",
         866 => x"04040404",
         867 => x"04040410",
         868 => x"10101010",
         869 => x"10104141",
         870 => x"41414141",
         871 => x"01010101",
         872 => x"01010101",
         873 => x"01010101",
         874 => x"01010101",
         875 => x"01010101",
         876 => x"10101010",
         877 => x"10104242",
         878 => x"42424242",
         879 => x"02020202",
         880 => x"02020202",
         881 => x"02020202",
         882 => x"02020202",
         883 => x"02020202",
         884 => x"10101010",
         885 => x"20000000",
         886 => x"00000000",
         887 => x"00000000",
         888 => x"00000000",
         889 => x"00000000",
         890 => x"00000000",
         891 => x"00000000",
         892 => x"00000000",
         893 => x"00000000",
         894 => x"00000000",
         895 => x"00000000",
         896 => x"00000000",
         897 => x"00000000",
         898 => x"00000000",
         899 => x"00000000",
         900 => x"00000000",
         901 => x"00000000",
         902 => x"00000000",
         903 => x"00000000",
         904 => x"00000000",
         905 => x"00000000",
         906 => x"00000000",
         907 => x"00000000",
         908 => x"00000000",
         909 => x"00000000",
         910 => x"00000000",
         911 => x"00000000",
         912 => x"00000000",
         913 => x"00000000",
         914 => x"00000000",
         915 => x"00000000",
         916 => x"00000000",
         917 => x"00000000",
         918 => x"18000020",
         919 => x"a80c0010",
         920 => x"b40c0010",
         921 => x"e00c0010",
         922 => x"0c0d0010",
         923 => x"340d0010",
         924 => x"00000000",
         925 => x"00000000",
         926 => x"00000000",
         927 => x"00000000",
         928 => x"00000000",
         929 => x"00000000",
         930 => x"00000000",
         931 => x"00000000",
         932 => x"00000000",
         933 => x"00000000",
         934 => x"00000000",
         935 => x"00000000",
         936 => x"00000000",
         937 => x"00000000",
         938 => x"00000000",
         939 => x"00000000",
         940 => x"00000000",
         941 => x"00000000",
         942 => x"00000000",
         943 => x"00000000",
         944 => x"00000000",
         945 => x"00000000",
         946 => x"00000000",
         947 => x"00000000",
         948 => x"00000000",
         949 => x"18000020",
         others => (others => '0')
        );

begin

    gen_bootrom: if HAVE_BOOTLOADER_ROM generate
        O_instruction_misaligned_error <= '0' when I_pc(1 downto 0) = "00" else '1';        

        -- ROM, for both instructions and read-only data
        process (I_clk, I_areset, I_pc, I_memaddress, I_csboot, I_memsize, I_stall) is
        variable address_instr : integer range 0 to bootloader_size-1;
        variable address_data : integer range 0 to bootloader_size-1;
        variable instr_var : data_type;
        variable instr_recode : data_type;
        variable romdata_var : data_type;
        constant x : data_type := (others => 'X');
        begin
            -- Calculate addresses
            address_instr := to_integer(unsigned(I_pc(bootloader_size_bits-1 downto 2)));
            address_data := to_integer(unsigned(I_memaddress(bootloader_size_bits-1 downto 2)));

            -- Quartus will detect ROM table and uses onboard RAM
            -- Do not use reset, otherwise ROM will be created with ALMs
            if rising_edge(I_clk) then
                if I_stall = '0' then
                    instr_var := bootrom(address_instr);
                end if;
                romdata_var := bootrom(address_data);
            end if;
            
            -- Recode instruction
            O_instr <= instr_var(7 downto 0) & instr_var(15 downto 8) & instr_var(23 downto 16) & instr_var(31 downto 24);
            
            O_load_misaligned_error <= '0';
            
            -- By natural size, for data
            if I_csboot = '1' then
                if I_memsize = memsize_word and I_memaddress(1 downto 0) = "00" then
                    O_dataout <= romdata_var(7 downto 0) & romdata_var(15 downto 8) & romdata_var(23 downto 16) & romdata_var(31 downto 24);
                elsif I_memsize = memsize_halfword and I_memaddress(1 downto 0) = "00" then
                    O_dataout <= x(31 downto 16) & romdata_var(23 downto 16) & romdata_var(31 downto 24);
                elsif I_memsize = memsize_halfword and I_memaddress(1 downto 0) = "10" then
                    O_dataout <= x(31 downto 16) & romdata_var(7 downto 0) & romdata_var(15 downto 8);
                elsif I_memsize = memsize_byte then
                    case I_memaddress(1 downto 0) is
                        when "00" => O_dataout <= x(31 downto 8) & romdata_var(31 downto 24);
                        when "01" => O_dataout <= x(31 downto 8) & romdata_var(23 downto 16);
                        when "10" => O_dataout <= x(31 downto 8) & romdata_var(15 downto 8);
                        when "11" => O_dataout <= x(31 downto 8) & romdata_var(7 downto 0);
                        when others => O_dataout <= x; O_load_misaligned_error <= '1';
                    end case;
                else
                    -- Chip select, but not aligned
                    O_dataout <= x;
                    O_load_misaligned_error <= '1';
                end if;
            else
                -- No chip select, so no data
                O_dataout <= x;
            end if;
        end process;
    end generate;

    gen_bootrom_not: if not HAVE_BOOTLOADER_ROM generate
        O_instruction_misaligned_error <= '0';
        O_load_misaligned_error <= '0';
        O_dataout <= (others => 'X');
        O_instr  <= (others => 'X');
    end generate;
end architecture rtl;
