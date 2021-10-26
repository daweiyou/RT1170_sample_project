#include "../lv_examples.h"
#if LV_BUILD_EXAMPLES && LV_USE_IMG

/**
 * Using the Size, Position and Padding style properties
 */
void lv_example_style_1(void)
{
     static lv_style_t style;
     lv_style_init(&style);
     lv_style_set_radius(&style, 20);

     /*Make a gradient*/
     lv_style_set_width(&style, 150);
     lv_style_set_height(&style, LV_SIZE_CONTENT);

     lv_style_set_pad_ver(&style, 20);
     lv_style_set_pad_left(&style, 5);

     //lv_style_set_x(&style, lv_pct(50));//50% position
     //lv_style_set_y(&style, 80);

     /*Create an object with the new style*/
     lv_obj_t * obj = lv_obj_create(lv_scr_act());
     lv_obj_add_style(obj, &style, 0);
     lv_obj_t * label = lv_label_create(obj);
     lv_obj_set_pos(label, 10, 10);
     lv_obj_set_size(label, 120, 50);
     lv_label_set_text(label, "Hello");
     
     
     lv_obj_t * label1 = lv_label_create(lv_scr_act());
     lv_obj_remove_style_all(label1);
     lv_obj_add_style(label1, &style, 0);
     lv_obj_set_pos(label1, 10, 180);
     lv_obj_set_size(label1, 120, 50);
     lv_label_set_text(label1, "Hello");
}

#endif
