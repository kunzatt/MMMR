package com.ssafy.mmmr.news.scheduler;

import com.ssafy.mmmr.news.service.NewsService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

@Slf4j
@RequiredArgsConstructor
@Component
public class NewsScheduler {

    private final NewsService newsService;

    @Scheduled(cron = "0 * * * * *")
    public void scheduleNews() {
        newsService.newsCrawler();
    }
}
