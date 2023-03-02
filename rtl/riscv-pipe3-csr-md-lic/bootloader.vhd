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
-- # https:/github.com/jesseopdenbrouw/riscv-minimal                                               #
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
          I_address : in data_type;
          I_csboot : in std_logic;
          I_size : in memsize_type;
          I_stall : in std_logic;
          O_instr : out data_type;
          O_data_out : out data_type;
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
          22 => x"938545e4",
          23 => x"13050500",
          24 => x"ef004042",
          25 => x"ef009019",
          26 => x"b7050020",
          27 => x"13060000",
          28 => x"93850500",
          29 => x"13055000",
          30 => x"ef000047",
          31 => x"ef001014",
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
          58 => x"130909af",
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
         118 => x"130545b6",
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
         133 => x"13050cd3",
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
         183 => x"9389d9d3",
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
         218 => x"1306d6d3",
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
         315 => x"93050000",
         316 => x"1305101b",
         317 => x"232e1106",
         318 => x"232a9106",
         319 => x"23282107",
         320 => x"23263107",
         321 => x"23244107",
         322 => x"232c8106",
         323 => x"23225107",
         324 => x"23206107",
         325 => x"232e7105",
         326 => x"232c8105",
         327 => x"232a9105",
         328 => x"2328a105",
         329 => x"2326b105",
         330 => x"eff05fd1",
         331 => x"37150010",
         332 => x"1305c5b3",
         333 => x"eff09fd3",
         334 => x"b70700f0",
         335 => x"1307f03f",
         336 => x"b7091000",
         337 => x"3709a000",
         338 => x"23a2e700",
         339 => x"93041000",
         340 => x"9389f9ff",
         341 => x"370a00f0",
         342 => x"13091900",
         343 => x"b3f73401",
         344 => x"639c0700",
         345 => x"1305a002",
         346 => x"eff05fce",
         347 => x"83274a00",
         348 => x"93d71700",
         349 => x"2322fa00",
         350 => x"eff0dfb0",
         351 => x"13040500",
         352 => x"6310050e",
         353 => x"93841400",
         354 => x"e39a24fd",
         355 => x"b70400f0",
         356 => x"23a20400",
         357 => x"631c0400",
         358 => x"93050000",
         359 => x"13050000",
         360 => x"eff0dfc9",
         361 => x"23a20400",
         362 => x"e7000400",
         363 => x"eff09fae",
         364 => x"93071002",
         365 => x"93040000",
         366 => x"631cf51c",
         367 => x"37140010",
         368 => x"130504b6",
         369 => x"eff09fca",
         370 => x"370900f0",
         371 => x"130a3005",
         372 => x"930aa004",
         373 => x"130b3002",
         374 => x"93092000",
         375 => x"930ba000",
         376 => x"83274900",
         377 => x"93c71700",
         378 => x"2322f900",
         379 => x"eff09faa",
         380 => x"1375f50f",
         381 => x"63184517",
         382 => x"eff0dfa9",
         383 => x"137cf50f",
         384 => x"9307fcfc",
         385 => x"93f7f70f",
         386 => x"63e0f910",
         387 => x"93071003",
         388 => x"631cfc04",
         389 => x"13052000",
         390 => x"eff05fc9",
         391 => x"930cd5ff",
         392 => x"13054000",
         393 => x"eff09fc8",
         394 => x"370d01ff",
         395 => x"b70d0001",
         396 => x"130c0500",
         397 => x"b38cac00",
         398 => x"130dfdff",
         399 => x"938dfdff",
         400 => x"631a9c05",
         401 => x"130ca000",
         402 => x"eff0dfa4",
         403 => x"1375f50f",
         404 => x"e31c85ff",
         405 => x"130504b6",
         406 => x"eff05fc1",
         407 => x"6ff05ff8",
         408 => x"13041000",
         409 => x"6ff09ff2",
         410 => x"93072003",
         411 => x"13052000",
         412 => x"631afc00",
         413 => x"eff09fc3",
         414 => x"930cc5ff",
         415 => x"13056000",
         416 => x"6ff05ffa",
         417 => x"eff09fc2",
         418 => x"930cb5ff",
         419 => x"13058000",
         420 => x"6ff05ff9",
         421 => x"1376ccff",
         422 => x"13052000",
         423 => x"2326c100",
         424 => x"eff0dfc0",
         425 => x"0326c100",
         426 => x"93070500",
         427 => x"b706ffff",
         428 => x"13753c00",
         429 => x"03270600",
         430 => x"93053000",
         431 => x"13081000",
         432 => x"9386f60f",
         433 => x"63063503",
         434 => x"630ab502",
         435 => x"630c0501",
         436 => x"137707f0",
         437 => x"b3e7e700",
         438 => x"2320f600",
         439 => x"130c1c00",
         440 => x"6ff01ff6",
         441 => x"3377d700",
         442 => x"93978700",
         443 => x"6ff09ffe",
         444 => x"3377a701",
         445 => x"93970701",
         446 => x"6ff0dffd",
         447 => x"3377b701",
         448 => x"93978701",
         449 => x"6ff01ffd",
         450 => x"93079cfc",
         451 => x"93f7f70f",
         452 => x"63e2f904",
         453 => x"13052000",
         454 => x"eff05fb9",
         455 => x"93077003",
         456 => x"13058000",
         457 => x"630afc00",
         458 => x"93078003",
         459 => x"13056000",
         460 => x"6304fc00",
         461 => x"13054000",
         462 => x"eff05fb7",
         463 => x"93040500",
         464 => x"130ca000",
         465 => x"eff01f95",
         466 => x"1375f50f",
         467 => x"e31c85ff",
         468 => x"6ff05ff0",
         469 => x"eff01f94",
         470 => x"1375f50f",
         471 => x"e31c75ff",
         472 => x"6ff05fef",
         473 => x"631a5509",
         474 => x"130504b6",
         475 => x"eff01fb0",
         476 => x"93050000",
         477 => x"13050000",
         478 => x"eff05fac",
         479 => x"23220900",
         480 => x"e7800400",
         481 => x"b70700f0",
         482 => x"1307a00a",
         483 => x"23a2e700",
         484 => x"37190010",
         485 => x"130549b6",
         486 => x"b7190010",
         487 => x"eff01fad",
         488 => x"13040000",
         489 => x"371b0010",
         490 => x"b71b0010",
         491 => x"9389d9d3",
         492 => x"b7170010",
         493 => x"138587b6",
         494 => x"eff05fab",
         495 => x"93059002",
         496 => x"13054101",
         497 => x"eff0df8e",
         498 => x"13054101",
         499 => x"ef00c02c",
         500 => x"b7170010",
         501 => x"130a0500",
         502 => x"9385c7b6",
         503 => x"13054101",
         504 => x"eff05fce",
         505 => x"631e0500",
         506 => x"37150010",
         507 => x"130505b7",
         508 => x"eff0dfa7",
         509 => x"6f004003",
         510 => x"e31e65e5",
         511 => x"6ff09ff8",
         512 => x"b7170010",
         513 => x"9385c7c5",
         514 => x"13054101",
         515 => x"eff09fcb",
         516 => x"63100502",
         517 => x"93050000",
         518 => x"eff05fa2",
         519 => x"b70700f0",
         520 => x"23a20700",
         521 => x"e7800400",
         522 => x"e3040af8",
         523 => x"6f004018",
         524 => x"b7170010",
         525 => x"13063000",
         526 => x"938507c6",
         527 => x"13054101",
         528 => x"ef004027",
         529 => x"63100504",
         530 => x"93050000",
         531 => x"13057101",
         532 => x"eff01fb1",
         533 => x"93773500",
         534 => x"13040500",
         535 => x"63940706",
         536 => x"93058000",
         537 => x"eff05fbb",
         538 => x"37150010",
         539 => x"130545c6",
         540 => x"eff0df9f",
         541 => x"03250400",
         542 => x"93058000",
         543 => x"eff0dfb9",
         544 => x"6ff09ffa",
         545 => x"13063000",
         546 => x"93050bc8",
         547 => x"13054101",
         548 => x"ef004022",
         549 => x"631e0502",
         550 => x"93050101",
         551 => x"13057101",
         552 => x"eff01fac",
         553 => x"93773500",
         554 => x"13040500",
         555 => x"639c0700",
         556 => x"03250101",
         557 => x"93050000",
         558 => x"eff09faa",
         559 => x"2320a400",
         560 => x"6ff09ff6",
         561 => x"37150010",
         562 => x"130585c6",
         563 => x"6ff05ff2",
         564 => x"13063000",
         565 => x"93854bc8",
         566 => x"13054101",
         567 => x"ef00801d",
         568 => x"83474101",
         569 => x"1307e006",
         570 => x"630c0508",
         571 => x"639ae70a",
         572 => x"93773400",
         573 => x"e39807fc",
         574 => x"130c0404",
         575 => x"b71c0010",
         576 => x"371d0010",
         577 => x"930d80ff",
         578 => x"93058000",
         579 => x"13050400",
         580 => x"eff09fb0",
         581 => x"13854cc6",
         582 => x"eff05f95",
         583 => x"83270400",
         584 => x"93058000",
         585 => x"130a8001",
         586 => x"13850700",
         587 => x"2326f100",
         588 => x"eff09fae",
         589 => x"13058dc8",
         590 => x"eff05f93",
         591 => x"b70a00ff",
         592 => x"8327c100",
         593 => x"33f55701",
         594 => x"33554501",
         595 => x"b3063501",
         596 => x"83c60600",
         597 => x"93f67609",
         598 => x"63800604",
         599 => x"130a8aff",
         600 => x"eff0df8e",
         601 => x"93da8a00",
         602 => x"e31cbafd",
         603 => x"13044400",
         604 => x"130549b6",
         605 => x"eff09f8f",
         606 => x"e31884f9",
         607 => x"6ff05fe3",
         608 => x"e388e7f6",
         609 => x"93050000",
         610 => x"13057101",
         611 => x"eff05f9d",
         612 => x"13040500",
         613 => x"6ff0dff5",
         614 => x"1305e002",
         615 => x"6ff01ffc",
         616 => x"e3080ae0",
         617 => x"37150010",
         618 => x"1305c5c8",
         619 => x"eff01f8c",
         620 => x"130549b6",
         621 => x"eff09f8b",
         622 => x"6ff09fdf",
         623 => x"130101ff",
         624 => x"23248100",
         625 => x"23261100",
         626 => x"93070000",
         627 => x"13040500",
         628 => x"63880700",
         629 => x"93050000",
         630 => x"97000000",
         631 => x"e7000000",
         632 => x"b7170010",
         633 => x"03a507e4",
         634 => x"83278502",
         635 => x"63840700",
         636 => x"e7800700",
         637 => x"13050400",
         638 => x"eff0dfae",
         639 => x"130101ff",
         640 => x"23248100",
         641 => x"23229100",
         642 => x"37140010",
         643 => x"b7140010",
         644 => x"938744e4",
         645 => x"130444e4",
         646 => x"3304f440",
         647 => x"23202101",
         648 => x"23261100",
         649 => x"13542440",
         650 => x"938444e4",
         651 => x"13090000",
         652 => x"63108904",
         653 => x"b7140010",
         654 => x"37140010",
         655 => x"938744e4",
         656 => x"130444e4",
         657 => x"3304f440",
         658 => x"13542440",
         659 => x"938444e4",
         660 => x"13090000",
         661 => x"63188902",
         662 => x"8320c100",
         663 => x"03248100",
         664 => x"83244100",
         665 => x"03290100",
         666 => x"13010101",
         667 => x"67800000",
         668 => x"83a70400",
         669 => x"13091900",
         670 => x"93844400",
         671 => x"e7800700",
         672 => x"6ff01ffb",
         673 => x"83a70400",
         674 => x"13091900",
         675 => x"93844400",
         676 => x"e7800700",
         677 => x"6ff01ffc",
         678 => x"93070500",
         679 => x"03c70700",
         680 => x"93871700",
         681 => x"e31c07fe",
         682 => x"3385a740",
         683 => x"1305f5ff",
         684 => x"67800000",
         685 => x"630a0602",
         686 => x"1306f6ff",
         687 => x"13070000",
         688 => x"b307e500",
         689 => x"b386e500",
         690 => x"83c70700",
         691 => x"83c60600",
         692 => x"6398d700",
         693 => x"6306c700",
         694 => x"13071700",
         695 => x"e39207fe",
         696 => x"3385d740",
         697 => x"67800000",
         698 => x"13050000",
         699 => x"67800000",
         700 => x"14020010",
         701 => x"58010010",
         702 => x"58010010",
         703 => x"58010010",
         704 => x"58010010",
         705 => x"b8010010",
         706 => x"58010010",
         707 => x"cc010010",
         708 => x"58010010",
         709 => x"58010010",
         710 => x"cc010010",
         711 => x"58010010",
         712 => x"58010010",
         713 => x"58010010",
         714 => x"58010010",
         715 => x"58010010",
         716 => x"58010010",
         717 => x"58010010",
         718 => x"2c010010",
         719 => x"0d0a5448",
         720 => x"55415320",
         721 => x"52495343",
         722 => x"2d562042",
         723 => x"6f6f746c",
         724 => x"6f616465",
         725 => x"72207630",
         726 => x"2e320d0a",
         727 => x"00000000",
         728 => x"3f0a0000",
         729 => x"0d0a0000",
         730 => x"3e200000",
         731 => x"68000000",
         732 => x"48656c70",
         733 => x"3a0d0a20",
         734 => x"68202020",
         735 => x"20202020",
         736 => x"20202020",
         737 => x"20202020",
         738 => x"202d2074",
         739 => x"68697320",
         740 => x"68656c70",
         741 => x"0d0a2072",
         742 => x"20202020",
         743 => x"20202020",
         744 => x"20202020",
         745 => x"20202020",
         746 => x"2d207275",
         747 => x"6e206170",
         748 => x"706c6963",
         749 => x"6174696f",
         750 => x"6e0d0a20",
         751 => x"7277203c",
         752 => x"61646472",
         753 => x"3e202020",
         754 => x"20202020",
         755 => x"202d2072",
         756 => x"65616420",
         757 => x"776f7264",
         758 => x"2066726f",
         759 => x"6d206164",
         760 => x"64720d0a",
         761 => x"20777720",
         762 => x"3c616464",
         763 => x"723e203c",
         764 => x"64617461",
         765 => x"3e202d20",
         766 => x"77726974",
         767 => x"6520776f",
         768 => x"72642064",
         769 => x"61746120",
         770 => x"61742061",
         771 => x"6464720d",
         772 => x"0a206477",
         773 => x"203c6164",
         774 => x"64723e20",
         775 => x"20202020",
         776 => x"2020202d",
         777 => x"2064756d",
         778 => x"70203136",
         779 => x"20776f72",
         780 => x"64730d0a",
         781 => x"206e2020",
         782 => x"20202020",
         783 => x"20202020",
         784 => x"20202020",
         785 => x"20202d20",
         786 => x"64756d70",
         787 => x"206e6578",
         788 => x"74203136",
         789 => x"20776f72",
         790 => x"64730000",
         791 => x"72000000",
         792 => x"72772000",
         793 => x"3a200000",
         794 => x"4e6f7420",
         795 => x"6f6e2034",
         796 => x"2d627974",
         797 => x"6520626f",
         798 => x"756e6461",
         799 => x"72792100",
         800 => x"77772000",
         801 => x"64772000",
         802 => x"20200000",
         803 => x"3f3f0000",
         804 => x"626f6f74",
         805 => x"6c6f6164",
         806 => x"65720000",
         807 => x"54485541",
         808 => x"53205249",
         809 => x"53432d56",
         810 => x"20525633",
         811 => x"32494d20",
         812 => x"62617265",
         813 => x"206d6574",
         814 => x"616c2070",
         815 => x"726f6365",
         816 => x"73736f72",
         817 => x"00000000",
         818 => x"54686520",
         819 => x"48616775",
         820 => x"6520556e",
         821 => x"69766572",
         822 => x"73697479",
         823 => x"206f6620",
         824 => x"4170706c",
         825 => x"69656420",
         826 => x"53636965",
         827 => x"6e636573",
         828 => x"00000000",
         829 => x"44657061",
         830 => x"72746d65",
         831 => x"6e74206f",
         832 => x"6620456c",
         833 => x"65637472",
         834 => x"6963616c",
         835 => x"20456e67",
         836 => x"696e6565",
         837 => x"72696e67",
         838 => x"00000000",
         839 => x"4a2e452e",
         840 => x"4a2e206f",
         841 => x"70206465",
         842 => x"6e204272",
         843 => x"6f757700",
         844 => x"3c627265",
         845 => x"616b3e0d",
         846 => x"0a000000",
         847 => x"00202020",
         848 => x"20202020",
         849 => x"20202828",
         850 => x"28282820",
         851 => x"20202020",
         852 => x"20202020",
         853 => x"20202020",
         854 => x"20202020",
         855 => x"20881010",
         856 => x"10101010",
         857 => x"10101010",
         858 => x"10101010",
         859 => x"10040404",
         860 => x"04040404",
         861 => x"04040410",
         862 => x"10101010",
         863 => x"10104141",
         864 => x"41414141",
         865 => x"01010101",
         866 => x"01010101",
         867 => x"01010101",
         868 => x"01010101",
         869 => x"01010101",
         870 => x"10101010",
         871 => x"10104242",
         872 => x"42424242",
         873 => x"02020202",
         874 => x"02020202",
         875 => x"02020202",
         876 => x"02020202",
         877 => x"02020202",
         878 => x"10101010",
         879 => x"20000000",
         880 => x"00000000",
         881 => x"00000000",
         882 => x"00000000",
         883 => x"00000000",
         884 => x"00000000",
         885 => x"00000000",
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
         912 => x"18000020",
         913 => x"900c0010",
         914 => x"9c0c0010",
         915 => x"c80c0010",
         916 => x"f40c0010",
         917 => x"1c0d0010",
         918 => x"00000000",
         919 => x"00000000",
         920 => x"00000000",
         921 => x"00000000",
         922 => x"00000000",
         923 => x"00000000",
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
         943 => x"18000020",
         others => (others => '0')
        );

begin

    gen_bootrom: if HAVE_BOOTLOADER_ROM generate
        O_instruction_misaligned_error <= '0' when I_pc(1 downto 0) = "00" else '1';        

        -- ROM, for both instructions and read-only data
        process (I_clk, I_areset, I_pc, I_address, I_csboot, I_size, I_stall) is
        variable address_instr : integer range 0 to bootloader_size-1;
        variable address_data : integer range 0 to bootloader_size-1;
        variable instr_var : data_type;
        variable instr_recode : data_type;
        variable romdata_var : data_type;
        constant x : data_type := (others => 'X');
        begin
            -- Calculate addresses
            address_instr := to_integer(unsigned(I_pc(bootloader_size_bits-1 downto 2)));
            address_data := to_integer(unsigned(I_address(bootloader_size_bits-1 downto 2)));

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
                if I_size = memsize_word and I_address(1 downto 0) = "00" then
                    O_data_out <= romdata_var(7 downto 0) & romdata_var(15 downto 8) & romdata_var(23 downto 16) & romdata_var(31 downto 24);
                elsif I_size = memsize_halfword and I_address(1 downto 0) = "00" then
                    O_data_out <= x(31 downto 16) & romdata_var(23 downto 16) & romdata_var(31 downto 24);
                elsif I_size = memsize_halfword and I_address(1 downto 0) = "10" then
                    O_data_out <= x(31 downto 16) & romdata_var(7 downto 0) & romdata_var(15 downto 8);
                elsif I_size = memsize_byte then
                    case I_address(1 downto 0) is
                        when "00" => O_data_out <= x(31 downto 8) & romdata_var(31 downto 24);
                        when "01" => O_data_out <= x(31 downto 8) & romdata_var(23 downto 16);
                        when "10" => O_data_out <= x(31 downto 8) & romdata_var(15 downto 8);
                        when "11" => O_data_out <= x(31 downto 8) & romdata_var(7 downto 0);
                        when others => O_data_out <= x; O_load_misaligned_error <= '1';
                    end case;
                else
                    -- Chip select, but not aligned
                    O_data_out <= x;
                    O_load_misaligned_error <= '1';
                end if;
            else
                -- No chip select, so no data
                O_data_out <= x;
            end if;
        end process;
    end generate;

    gen_bootrom_not: if not HAVE_BOOTLOADER_ROM generate
        O_instruction_misaligned_error <= '0';
        O_load_misaligned_error <= '0';
        O_data_out <= (others => 'X');
        O_instr  <= (others => 'X');
    end generate;
end architecture rtl;
