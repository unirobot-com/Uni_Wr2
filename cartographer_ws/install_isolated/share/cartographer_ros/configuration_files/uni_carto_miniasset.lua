options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.01,
      max_range = 2.0,
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filestem = "small_map",
      resolution = 0.01,
    }
  }
}

return options
