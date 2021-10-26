#include "../lv_examples.h"
#if LV_BUILD_EXAMPLES && LV_USE_SLIDER

static lv_obj_t * label;

static void slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);

    switch(code)
    {
    case LV_EVENT_VALUE_CHANGED:
    /*Refresh the text*/
    lv_label_set_text_fmt(label, "%d", lv_slider_get_value(slider));
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
    break;
    
    case LV_EVENT_PRESSED:
    /*Refresh the text*/
    //lv_label_set_text_fmt(label, "%d", lv_slider_get_value(slider));
    lv_label_set_text(label, "Pressed");
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
      break;
      
      
    case LV_EVENT_RELEASED:
    /*Refresh the text*/
    //lv_label_set_text_fmt(label, "%d", lv_slider_get_value(slider));
    lv_label_set_text(label, "Rel");
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
      break;
    
    }
}

/**
 * Create a slider and write its value on a label.
 */
void lv_example_get_started_3(void)
{
    /*Create a slider in the center of the display*/
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_obj_set_width(slider, 200);                          /*Set the width*/
    lv_obj_center(slider);                                  /*Align to the center of the parent (screen)*/
    //lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED | LV_EVENT_PRESSED | LV_EVENT_RELEASED, NULL);     /*Assign an event function*/
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_ALL, NULL); 

    /*Create a label below the slider*/
    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "0");
    lv_obj_align_to(label, slider, LV_ALIGN_OUT_TOP_MID, 0, -15);    /*Align top of the slider*/
}

#endif

