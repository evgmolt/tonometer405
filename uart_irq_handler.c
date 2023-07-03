void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

    if(__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE) != RESET)
    {
        uart2_buff[uart2_count] = (uint8_t)(USART2->DR & 0x00FF);
        uart2_count++;
        if (uart2_count = UART_BUF_SIZE) uart2_count = 0;
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    }    

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

